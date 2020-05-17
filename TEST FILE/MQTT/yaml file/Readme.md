Code to use with Home Assistant.
Write by Jussip see:
https://forum.ardumower.de/threads/arctic-hare-mod-by-jussi.23557/post-40475

Create into your hass config directory a folder named package
Into your main configuration.yaml add

homeassistant:
  packages: !include_dir_named package

And into Lovelace:
At the end of the raw data add the content of lovelace.yaml.
