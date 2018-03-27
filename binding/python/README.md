This documents lists some build options related to the Python bindings and additional commands provided.

# Available options

- `PYTHON_BINDING`: if `ON`, builds the bindings
- `PYTHON_BINDING_USER_INSTALL`: if `ON`, this will install the bindings in the user directory rather than system directory (i.e. run `pip install --user` rather than `pip install`)

# Additional commands

Incremental builds of the bindings might not be triggered properly, you might face two issues:
1. Bindings are re-compiled
2. Bindings are not installed

This will result in your bindings not working.

Two targets are provided to overcome such issues:
1. `make force-install-mc-rtc-python-bindings` will force a new installation
2. `make force-mc-rtc-python-bindings` will force a rebuild of the bindings
