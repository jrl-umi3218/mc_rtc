from .._mc_rtc import @MODULE_NAME@ as _submodule

# Lift the submodule attributes into this folder's namespace
_exported = []
for _attr in dir(_submodule):
    if not _attr.startswith("__"):
        globals()[_attr] = getattr(_submodule, _attr)
        _exported.append(_attr)

__all__ = _exported

# print(__all__)

# # Cleanup to keep the namespace clean
# del _submodule
# del _attr
# del _exported