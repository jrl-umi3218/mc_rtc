In mc_rtc, every interface uses the {% doxygen mc_control::MCGlobalController %} class to initialize and run the controllers in the framework. In particular, they will run the {% doxygen mc_control::MCGlobalController::run() %} function on every iteration.

The plugin system allows one to write a component that will run at the start and/or end of this run function.

These plugins can be used for a variety of purpose:

- Publish data using a 3rd-party middleware (for example, the ROS plugin);
- Provide sensor data that is obtained with a different interface than the one where mc_rtc is running;
- Provide high-level functionalities that are above a single controller scope;

The [new plugin tutorial]({{site.baseurl}}/tutorials/advanced/new-plugin.html) provides more information on writing plugins for mc_rtc.

The plugins' documentation should provide information about available options and usage. This tutorial is focused on enabling and configuring these plugins.

## Enabling plugins at the global level

Plugins can be added to the `Plugins` list in [mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html#possible-locations-for-mc-rtc-configuration). For example:

```yaml
Plugins: [Joystick, OculusVR]
```

The plugins listed in this configuration are enabled regardless of the controller that is selected.

<div class="alert alert-info">When installing a plugin, an option can be provided to enable automatic loading. Such plugins are loaded regarldess of their presence in the `Plugins` entry.</div>

### Configuration locations considered in this case

The following two files will be read by mc_rtc (if they exist):

<ol>
  <li>{% ihighlight bash %}${PLUGIN_RUNTIME_INSTALL_PREFIX}/etc/${PLUGIN_NAME}.yaml{% endihighlight %}</li>
  <li>
    Linux/macOS: {% ihighlight bash %}$HOME/.config/mc_rtc/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}<br/>
    Windows: {% ihighlight msshell %}%APPDATA%/mc_rtc/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}
  </li>
</ol>

Where `${PLUGIN_RUNTIME_INSTALL_PREFIX}` is `${INSTALL_PREFIX}/lib/mc_plugins` on Linux/macOS and `${INSTALL_PREFIX}/bin/mc_plugins` on Windows.

## Enabling plugins at the controller level

Plugins can be enabled for a given controller by adding a `Plugins` entry in the controller's configuration.

### Configuration locations considered in this case

<div class="alert alert-warning">Those files are only considered for the plugin configuration if the plugin is enabled at the controller level.</div>

In addition to the global configuration files, the following files are also considered for the plugin configuration when it is enabled by a controller (if they exist):

<ol>
  <li>{% ihighlight bash %}${CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/${CONTROLLER_NAME}/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}</li>
  <li>
    Linux/macOS: {% ihighlight bash %}$HOME/.config/mc_rtc/controllers/${CONTROLLER_NAME}/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}<br/>
    Windows: {% ihighlight msshell %}%APPDATA%/mc_rtc/controllers/${CONTROLLER_NAME}/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}
  </li>
</ol>

Where `${CONTROLLER_RUNTIME_INSTALL_PREFIX}` is `${INSTALL_PREFIX}/lib/mc_controllers` on Linux/macOS and `${INSTALL_PREFIX}/bin/mc_controllers` on Windows.

<div class="alert alert-warning">When multiple controllers are enabled, if they use the same plugin then all these files are loaded together in the order the controllers appear in mc_rtc `Enabled` configuration entry.</div>
