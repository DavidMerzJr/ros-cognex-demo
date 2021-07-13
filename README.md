# Cognex Demo

## Block Diagram
----------------
![Block Diagram](block_diagram.png)

## Further Work
---------------
To substitute the basler for a cognex
- Change the expected number of points being loaded from the plc (basically do a find and replace for `20`)
- Make a simple node to publish a CameraInfo message with the cognex intrinsics
- Update the relative positions in either the plc_initializer node or directly in the PLC
