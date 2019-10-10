Technical notes on MessagePack format for GUIState
===

Notations
---

```
"..."   # String

{ ... } # Map

[ ... ] # Array

( ... ) # Optional
```

Message-format
---

For the root:

```
[ PROTOCOL_VERSION, { StaticData }, [ Sub-categories ] ]
```

For a sub-category:

```
[ "CategoryName", Widget1, ..., WidgetN, [ Sub-categories ] ]
```

For a given Widget:

```
[ "WidgetName", WidgetType, WidgetStackID, ( WidgetData ), ( WidgetConfiguration ) ]
```

`WidgetStackID` is `nil` if the widget does not belong to a specific stack.

The creation of widgets binary data is (mostly) performed in [include/mc\_rtc/GUIState.h](../../include/mc_rtc/GUIState.h). The common part is written by `GUIState` during the GUI update call.

The interpretation of widgets binary data is performed in [src/mc\_control/ControllerClient.cpp](../mc_control/ControllerClient.cpp).

For a `Form`, `WidgetData` is:

```
[ FormWidget1, ..., FormWidgetN ]
```

Where `FormWiget` is:

```
[ "FormWidgetName", FormWidgetType, FormWidgetRequired, (FormWidgetData) ]
```
