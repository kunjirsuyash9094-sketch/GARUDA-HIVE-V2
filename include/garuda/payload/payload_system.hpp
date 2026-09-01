#pragma once
#include "core/math_types.hpp"
#include "garuda/payload/payload_catalogue.hpp"
#include "garuda/payload/inspection_camera.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace garuda {

using dronesim::Vec3d;

/**
 * @brief Authoritative Modular Payload System for GARUDA-HL-01.
 * 
 * Manages physical coupling, mass/CoM/inertia calculations via Parallel Axis Theorem,
 * electrical load ledger, and the payload lifecycle state machine.
 */
class PayloadSystem {
public:
    static constexpr double DRY_MASS_BASELINE_KG = 8.50;
    static constexpr double MTOW_LIMIT_KG        = 15.00;
    static constexpr double MAX_PAYLOAD_LIMIT_KG = 6.50;

    PayloadSystem() noexcept {
        // Default canonical payload: High-Definition Inspection Camera
        attach_payload(PayloadType::INSPECTION_CAMERA);
    }

    /**
     * @brief Requests physical attachment of a modular payload.
     * 
     * Validates MTOW limit and executes state machine transition:
     * AVAILABLE -> ATTACHING -> ATTACHED -> ACTIVE
     * 
     * @param type Payload type from centralized catalogue
     * @return true if successfully attached, false if rejected due to MTOW or invalid type
     */
    bool attach_payload(PayloadType type) noexcept {
        if (type == PayloadType::NONE) {
            detach_payload();
            return true;
        }

        PayloadDescriptor candidate = PayloadCatalogue::get_descriptor(type);

        // MTOW Enforcement: Dry mass + candidate mass <= MTOW
        if (DRY_MASS_BASELINE_KG + candidate.mass_kg > MTOW_LIMIT_KG + 1e-4) {
            _current.state = PayloadState::FAULT;
            _current.health = 2; // FAULT
            _last_error = "Attachment rejected: Exceeds MTOW of " + std::to_string(MTOW_LIMIT_KG) + " kg";
            return false;
        }

        _current = candidate;
        _current.state = PayloadState::ATTACHED;
        _current.enabled = true;
        _current.health = 0;
        _last_error.clear();

        if (type == PayloadType::INSPECTION_CAMERA) {
            _camera.reset();
            _camera.set_status(CameraStatus::STREAMING);
        }

        return true;
    }

    /**
     * @brief Requests physical detachment and release of current payload.
     * 
     * Executes state machine transition:
     * ATTACHED/ACTIVE -> DETACHING -> DETACHED
     * Restores exact dry mass baseline.
     */
    bool detach_payload() noexcept {
        _current = PayloadCatalogue::get_descriptor(PayloadType::NONE);
        _current.state = PayloadState::DETACHED;
        _current.enabled = false;
        _camera.set_status(CameraStatus::OFFLINE);
        _last_error.clear();
        return true;
    }

    /**
     * @brief Transitions payload state machine with validation.
     */
    bool transition_state(PayloadState target_state) noexcept {
        // Safe transition validation rules
        switch (target_state) {
            case PayloadState::AVAILABLE:
                if (_current.state == PayloadState::DETACHED || _current.state == PayloadState::FAULT) {
                    _current.state = target_state;
                    return true;
                }
                break;
            case PayloadState::ATTACHING:
                if (_current.state == PayloadState::AVAILABLE || _current.state == PayloadState::DETACHED) {
                    _current.state = target_state;
                    return true;
                }
                break;
            case PayloadState::ATTACHED:
                if (_current.state == PayloadState::ATTACHING || _current.state == PayloadState::ACTIVE) {
                    _current.state = target_state;
                    return true;
                }
                break;
            case PayloadState::ACTIVE:
                if (_current.state == PayloadState::ATTACHED) {
                    _current.state = target_state;
                    return true;
                }
                break;
            case PayloadState::DETACHING:
                if (_current.state == PayloadState::ATTACHED || _current.state == PayloadState::ACTIVE) {
                    _current.state = target_state;
                    return true;
                }
                break;
            case PayloadState::DETACHED:
                if (_current.state == PayloadState::DETACHING || _current.state == PayloadState::FAULT) {
                    _current.state = target_state;
                    return true;
                }
                break;
            case PayloadState::FAULT:
                _current.state = target_state;
                _current.health = 2;
                return true;
        }

        _last_error = "Invalid state transition requested";
        return false;
    }

    // -------------------------------------------------------------------------
    // Camera & Gimbal Controls Delegation
    // -------------------------------------------------------------------------
    void set_gimbal(double pitch_deg, double yaw_deg) noexcept {
        _camera.set_gimbal(pitch_deg, yaw_deg);
    }

    void set_zoom(double zoom) noexcept {
        _camera.set_zoom(zoom);
    }

    [[nodiscard]] const InspectionCamera& camera() const noexcept { return _camera; }
    [[nodiscard]] InspectionCamera& mutable_camera() noexcept { return _camera; }

    // -------------------------------------------------------------------------
    // Physical Couplings & Mass Properties
    // -------------------------------------------------------------------------
    [[nodiscard]] bool has_payload() const noexcept {
        return _current.type != PayloadType::NONE && _current.state != PayloadState::DETACHED;
    }

    [[nodiscard]] double payload_mass_kg() const noexcept {
        return has_payload() ? _current.mass_kg : 0.0;
    }

    [[nodiscard]] double effective_total_mass_kg(double dry_mass_kg) const noexcept {
        return dry_mass_kg + payload_mass_kg();
    }

    [[nodiscard]] double payload_power_w() const noexcept {
        if (!has_payload()) return 0.0;
        if (_current.type == PayloadType::INSPECTION_CAMERA) {
            return _camera.power_draw_w();
        }
        return _current.power_w;
    }

    [[nodiscard]] Vec3d com_offset_m() const noexcept {
        return has_payload() ? _current.com_offset_m : Vec3d{0.0, 0.0, 0.0};
    }

    /**
     * @brief Computes combined Center of Mass of vehicle + payload.
     * 
     * CoM_eff = (m_dry * CoM_dry + m_payload * CoM_payload) / (m_dry + m_payload)
     */
    [[nodiscard]] Vec3d effective_com(double dry_mass_kg, const Vec3d& dry_com = {0.0, 0.0, 0.0}) const noexcept {
        if (!has_payload() || _current.mass_kg <= 0.0) return dry_com;
        double m_tot = dry_mass_kg + _current.mass_kg;
        if (m_tot <= 1e-6) return dry_com;
        return (dry_com * dry_mass_kg + _current.com_offset_m * _current.mass_kg) / m_tot;
    }

    /**
     * @brief Computes combined principal diagonal inertia using Parallel Axis Theorem.
     * 
     * I_eff = I_dry + I_payload + m_dry * (d_dry^2 * E - d_dry x d_dry) + m_p * (d_p^2 * E - d_p x d_p)
     */
    [[nodiscard]] Vec3d effective_inertia_diag(double dry_mass_kg, const Vec3d& dry_inertia_diag) const noexcept {
        if (!has_payload() || _current.mass_kg <= 0.0) {
            return dry_inertia_diag;
        }

        const double m0 = dry_mass_kg;
        const double mp = _current.mass_kg;
        const double M  = m0 + mp;

        const Vec3d com_eff = effective_com(m0);
        const Vec3d r0 = Vec3d{0.0, 0.0, 0.0} - com_eff;
        const Vec3d rp = _current.com_offset_m - com_eff;

        // Parallel Axis Theorem for dry airframe about combined CoM
        const double d_ixx_0 = m0 * (r0.y * r0.y + r0.z * r0.z);
        const double d_iyy_0 = m0 * (r0.x * r0.x + r0.z * r0.z);
        const double d_izz_0 = m0 * (r0.x * r0.x + r0.y * r0.y);

        // Parallel Axis Theorem for payload about combined CoM
        const double d_ixx_p = mp * (rp.y * rp.y + rp.z * rp.z);
        const double d_iyy_p = mp * (rp.x * rp.x + rp.z * rp.z);
        const double d_izz_p = mp * (rp.x * rp.x + rp.y * rp.y);

        const Vec3d& Ip = _current.inertia_diag_kgm2;

        return Vec3d{
            dry_inertia_diag.x + Ip.x + d_ixx_0 + d_ixx_p,
            dry_inertia_diag.y + Ip.y + d_iyy_0 + d_iyy_p,
            dry_inertia_diag.z + Ip.z + d_izz_0 + d_izz_p
        };
    }

    [[nodiscard]] const PayloadDescriptor& current() const noexcept { return _current; }
    [[nodiscard]] PayloadState state() const noexcept { return _current.state; }
    [[nodiscard]] const std::string& last_error() const noexcept { return _last_error; }

private:
    PayloadDescriptor _current{};
    InspectionCamera  _camera{};
    std::string       _last_error{};
};

} // namespace garuda
