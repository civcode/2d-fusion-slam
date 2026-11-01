# Local demo maps

Place exported FUSMAP2D `.slammap` files here to display them via
`docker compose up --build demo`. The files are read-only in the container.
They are local data, not source assets or verified world-frame maps.

The map generation registry `CURRENT` and `generation-*.slammap` files may
also be placed here. The UI never mutates, promotes or rolls back maps.
