# Haustuer Automation System (HAS)

## This is draft, develop in progress

## settings for configuration.yaml

The Switch for mode selection:

```yaml
switch:
  - platform: mqtt
    name: "Klingel Modus"
    unique_id: klingel_mode
    state_topic: "/home/haustuer/klingel/status"
    command_topic: "/home/haustuer/klingel/mode"
    payload_on: "1"
    payload_off: "0"
    state_on: "1"
    state_off: "0"
```

The sensors for actual ringing and status
```yaml
sensor:
  - platform: mqtt
    unique_id: klingel_trigger
    name: "Klingel Trigger"
    state_topic: "/home/haustuer/klingel"

  - platform: mqtt
    name: "Klingel Status"
    unique_id: klingel_status
    state_topic: "/home/haustuer/klingel/status"
```

The button for opening the door
```yaml
button:
  - platform: mqtt
    unique_id: dooropener
    name: "Tueroeffner"
    command_topic: "/home/haustuer/klingel/dooropener"
    payload_press: "1"
```

Entity card:
```yaml
type: entities
entities:
  - entity: sensor.klingel_trigger
    name: Klingel
    secondary_info: last-updated
  - entity: switch.klingel_mode
    name: KlingelStumm
  - entity: button.dooropener
title: Haustür
```