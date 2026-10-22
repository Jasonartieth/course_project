"""Arm backends with one interface (see base.Backend). Imports are lazy so the real-arm machine does not need
Genesis and the Mac does not need the Quanser SDK."""


def make_backend(name, **kwargs):
    if name == "pal":
        from .pal_hw import PalQArmBackend
        return PalQArmBackend(**kwargs)
    if name == "genesis":
        from .sim_genesis import GenesisBackend
        return GenesisBackend(**kwargs)
    if name == "fake":
        from .fake import FakeBackend
        return FakeBackend(**kwargs)
    raise ValueError(f"unknown backend {name!r} (pal | genesis | fake)")
