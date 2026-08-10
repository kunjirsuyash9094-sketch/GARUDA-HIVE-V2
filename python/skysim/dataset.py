"""Load recorded runs into ML-ready datasets.

Dependency-light: works with numpy alone. If PyTorch is installed you also get a
torch.utils.data.Dataset via TorchRunDataset.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np

from .record import load_run


class SkySimDataset:
    """Imitation-learning view over one or more recorded runs.

    Yields (observation, action) pairs. Observation is the state vector, plus
    depth/rgb if the runs recorded them (returned as a dict when present).
    Uses transitions [obs_t -> action_t], i.e. obs[:-1] aligned with actions.
    """

    def __init__(self, run_dirs, keys=("state", "depth")):
        if isinstance(run_dirs, (str, Path)):
            run_dirs = _discover(run_dirs)
        self.keys = tuple(keys)
        self._obs = {k: [] for k in self.keys}
        self._act = []
        for rd in run_dirs:
            r = load_run(rd)
            n = int(r["manifest"]["n_steps"])
            if n == 0:
                continue
            self._act.append(r["actions"][:n])
            for k in self.keys:
                if k in r:
                    self._obs[k].append(r[k][:n])   # obs[:-1] aligned to actions
        self._act = np.concatenate(self._act) if self._act else np.zeros((0, 4), np.float32)
        for k in list(self._obs):
            self._obs[k] = np.concatenate(self._obs[k]) if self._obs[k] else None
        self.keys = tuple(k for k in self.keys if self._obs.get(k) is not None)

    def __len__(self):
        return len(self._act)

    def __getitem__(self, i):
        obs = {k: self._obs[k][i] for k in self.keys}
        if list(obs.keys()) == ["state"]:
            obs = obs["state"]
        return obs, self._act[i]

    def arrays(self):
        """Return the whole dataset as arrays: (obs_dict_or_array, actions)."""
        obs = {k: self._obs[k] for k in self.keys}
        if list(obs.keys()) == ["state"]:
            obs = obs["state"]
        return obs, self._act


def _discover(root):
    root = Path(root)
    return sorted(p.parent for p in root.rglob("manifest.json"))


def make_torch_dataset(run_dirs, keys=("state", "depth")):
    """Build a torch Dataset (import guarded)."""
    import torch
    from torch.utils.data import Dataset

    base = SkySimDataset(run_dirs, keys=keys)

    class TorchRunDataset(Dataset):
        def __len__(self):
            return len(base)

        def __getitem__(self, i):
            obs, act = base[i]
            if isinstance(obs, dict):
                obs = {k: torch.as_tensor(v) for k, v in obs.items()}
            else:
                obs = torch.as_tensor(obs)
            return obs, torch.as_tensor(act)

    return TorchRunDataset()
