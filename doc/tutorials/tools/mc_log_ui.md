---
layout: tutorials
---

A simple GUI is provided in mc\_rtc to read the logs after an experiment
has been performed. This tool is installed as `mc_log_ui`

Provided you have obtained a log (e.g. `~/my_log.bin`) then you can open it in the GUI with:

```bash
$ mc_log_ui ~/my_log.bin
```

It should look like this:

<img src="img/mc_log_ui.png" class="img-fluid" alt="mc_log_ui default" />

<em>Note: some menus and operations described in the next sections are only available if you have mc\_rtc Python bindings</em>

### Basic operations

The panels on the left and the right of the plot let you select which data to display. You can select a grouped entry (e.g. `LeftFootForceSensor`) or a single entry (e.g. `LeftForceSensors -> fz`).

The selector under the plot let you select the abscissa for the graph. It defaults to `t` (i.e. the controller time). The buttons next to this selector let you lock the X axis, the left side Y axis and the right side Y axis respectively. Otherwise the limits adapt to the selected data. Note that the GUI doesn't try to align the left and right side Y-axes even when the ranges are similar.

You can open a new graph by clicking the `+` button next to the current tab title.

### Changing the axis limits

The button above the abscissa selector let you manipulate the graph view.

For more precise manipulations, you can change the range of the X and Y axis by using the `Ctrl+A` shortcut to open an axis menu. Setting manual limits will automatically lock the appropriate axis.

### Common plots menu

This menu contains a selection of common and useful plots for joint-level inputs/outputs and force sensors. When applicable, a second menu will let you select which joints to display, whether to display the appropriate limits and whether to generate one plot per joint.

### User plots menu

During the development of a controller you probably plot the same graph for different iterations of your controller. To save time, you can save your data selection. This is called a "User plot".

To create such a plot, you must setup the plot once manually. Then go the `User plots` menu and select `Save current plot`. You will be prompted for a name. This will then create a new entry in the `User plots` menu with this name.

After loading a new log, select this new entry in the `User plots` menu and it will redo the same selection and styling (it doesn't save axis limits or abscissa selection as they are likely to change).

Such plots are saved in a JSON format under `$HOME/.config/mc_log_ui/custom_plots.json`. This file can be stored in your controller repository to setup the log ui quickly on a new machine. It can also be passed to the `mc_plot_logs` tool (see below).

### Robot menu

This menu lets you select with which robot the plot was generated. The main effect is to display user-friendly names in joint-related entries (e.g. instead of `qIn_0` you will get `qIn_R_HIP_P` with JVRC1 but `qIn_RLEG_JOINT0` with HRP2DRC.

The robot selection is saved upon change and restored when you open the UI again. Unless you work with a lot of robots it is likely you will change it once.

### Style menu

The style menu lets you set many styling options for your graph, it has several entries:

- `Graph`: lets you choose the line color and line style for each plot;
- `Grid`: lets you enable a grid aligned with the left and/or right axis;
- `Labels/Title/Fonts`: lets you add labels to the axes, a title to the graph and choose the police size for those;

_Note: style options are also saved when you save a user plot_

### Other shortcuts

Additional shortcuts are provided to navigate through tabs:

- `Ctrl+T`: create a new tab
- `Ctrl+W`: close the current tab
- `Ctrl+PageDown`: next tab
- `Ctrl+PageUp`: previous tab

### Saving a figure

After choosing the fields you want and setting up the graph as you wish you can export by clicking the save button:

<img src="img/mc_log_ui_save.png" class="img-fluid" alt="mc_log_ui save button" />

You can choose to save as a variety of formats. If you save to SVG (Scalable Vector Graphics) you can further edit the file in a vector graphics editor such as [Inkscape](https://inkscape.org).

### Automated figure plotting

The final tool introduced here is `mc_plot_logs`. It expects a binary log and a JSON file describing the logs that you wish to plot (the format is the same as user plots from `mc_log_ui`). It will output a figure for each plot in the provided file.
