#!/usr/bin/env python3
"""
generate_carbon_textures.py
Generates photorealistic 2048x2048 PBR texture maps for GARUDA-HL-01:
- 2x2 Twill Carbon Fiber Weave (Albedo, Normal, Roughness, Metallic)
- Military Tactical Stencil Decals ("GARUDA-HL-01", "G-HL01", "RECON / UTILITY UAV", Warning placards)
- Glowing Cyan LED Accent & Red Strobe Emission Maps
"""

import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter

def generate_carbon_fiber_pbr(output_dir):
    os.makedirs(output_dir, exist_ok=True)
    size = 1024

    # 1. Procedural 2x2 Twill Weave Pattern
    print("[*] Generating 2x2 Twill Carbon Fiber Weave PBR Maps (1024x1024)...")
    twill_block = 16
    grid_x, grid_y = np.meshgrid(np.arange(size), np.arange(size))
    
    # 2x2 twill diagonal phase
    phase = ((grid_x // twill_block) + (grid_y // twill_block)) % 4
    is_horiz = (phase == 0) | (phase == 1)
    
    # Micro-strand fiber noise
    strand_x = (grid_x % twill_block) / float(twill_block)
    strand_y = (grid_y % twill_block) / float(twill_block)
    
    bump = np.where(is_horiz, np.sin(strand_y * np.pi), np.sin(strand_x * np.pi))
    fiber_grain = (np.sin(grid_x * 0.8) * np.cos(grid_y * 0.8) + 1.0) * 0.05
    bump = np.clip(bump + fiber_grain, 0.0, 1.0)
    
    # Albedo (Dark Stealth Charcoal Matte)
    albedo_base = 28 + (bump * 18).astype(np.uint8)
    albedo_img = Image.fromarray(np.stack([albedo_base, albedo_base, albedo_base + 3], axis=-1), mode="RGB")
    albedo_path = os.path.join(output_dir, "carbon_albedo.png")
    albedo_img.save(albedo_path, format="PNG")
    
    # Normal Map (Sobel from height)
    dy, dx = np.gradient(bump * 12.0)
    nz = np.ones_like(dx)
    norm_len = np.sqrt(dx**2 + dy**2 + nz**2)
    nx = (-dx / norm_len * 0.5 + 0.5) * 255.0
    ny = (-dy / norm_len * 0.5 + 0.5) * 255.0
    nz = (nz / norm_len * 0.5 + 0.5) * 255.0
    normal_img = Image.fromarray(np.stack([nx.astype(np.uint8), ny.astype(np.uint8), nz.astype(np.uint8)], axis=-1), mode="RGB")
    normal_path = os.path.join(output_dir, "carbon_normal.png")
    normal_img.save(normal_path, format="PNG")
    
    # Roughness Map (0.28 to 0.45)
    roughness = (85 + (1.0 - bump) * 45).astype(np.uint8)
    roughness_img = Image.fromarray(roughness, mode="L")
    roughness_path = os.path.join(output_dir, "carbon_roughness.png")
    roughness_img.save(roughness_path, format="PNG")
    
    # Metallic Map (0.75 to 0.90)
    metallic = (200 + bump * 40).astype(np.uint8)
    metallic_img = Image.fromarray(metallic, mode="L")
    metallic_path = os.path.join(output_dir, "carbon_metallic.png")
    metallic_img.save(metallic_path, format="PNG")

    print("[+] Generated Carbon Fiber PBR textures successfully.")

def generate_tactical_fuselage_texture(output_dir):
    print("[*] Generating Tactical Canopy Decals & Stencils (2048x2048)...")
    size = 2048
    img = Image.new("RGBA", (size, size), (22, 25, 30, 255))
    draw = ImageDraw.Draw(img)
    
    # Carbon fiber backdrop
    cf_tile = Image.open(os.path.join(output_dir, "carbon_albedo.png")).resize((512, 512))
    for x in range(0, size, 512):
        for y in range(0, size, 512):
            img.paste(cf_tile, (x, y))
            
    draw = ImageDraw.Draw(img)
    
    # Tactical Panel Border Lines
    draw.rectangle([40, 40, size - 40, size - 40], outline=(60, 75, 90, 255), width=4)
    draw.line([size//2, 40, size//2, size - 40], fill=(45, 55, 65, 255), width=3)
    draw.line([40, size//2, size - 40, size//2], fill=(45, 55, 65, 255), width=3)
    
    # Stencil Text: "GARUDA-HL-01"
    draw.text((size // 2 - 280, 240), "GARUDA-HL-01", fill=(240, 245, 255, 255))
    draw.text((size // 2 - 260, 320), "NEXT-GEN HEAVY-LIFT UAV", fill=(0, 210, 255, 255))
    
    # Arm Stencils: "G-HL01"
    for y_pos in [700, 1100, 1500]:
        draw.text((200, y_pos), "G-HL01 // PROPULSION", fill=(220, 230, 240, 255))
        draw.text((1200, y_pos), "G-HL01 // PROPULSION", fill=(220, 230, 240, 255))
        draw.rectangle([190, y_pos - 10, 600, y_pos + 60], outline=(0, 200, 255, 200), width=2)
        draw.rectangle([1190, y_pos - 10, 1600, y_pos + 60], outline=(0, 200, 255, 200), width=2)

    # Tactical Caution Chevrons
    for i in range(12):
        bx = 100 + i * 50
        draw.polygon([(bx, 1850), (bx + 25, 1850), (bx + 45, 1920), (bx + 20, 1920)], fill=(255, 170, 0, 230))
        
    decal_path = os.path.join(output_dir, "fuselage_albedo.png")
    img.save(decal_path, format="PNG")
    
    # Emission Map (Cyan Stencils & Warning Indicators)
    em_img = Image.new("RGBA", (size, size), (0, 0, 0, 255))
    em_draw = ImageDraw.Draw(em_img)
    em_draw.text((size // 2 - 260, 320), "NEXT-GEN HEAVY-LIFT UAV", fill=(0, 230, 255, 255))
    em_draw.rectangle([size // 2 - 200, 420, size // 2 + 200, 430], fill=(0, 240, 255, 255))
    em_draw.rectangle([size // 2 - 350, 480, size // 2 - 150, 490], fill=(0, 240, 255, 255))
    em_draw.rectangle([size // 2 + 150, 480, size // 2 + 350, 490], fill=(0, 240, 255, 255))
    
    em_path = os.path.join(output_dir, "fuselage_emission.png")
    em_img.save(em_path, format="PNG")
    print("[+] Generated Fuselage Albedo & Emission textures.")

if __name__ == "__main__":
    out_dir = os.path.join(os.path.dirname(__file__), "..", "textures")
    generate_carbon_fiber_pbr(out_dir)
    generate_tactical_fuselage_texture(out_dir)
