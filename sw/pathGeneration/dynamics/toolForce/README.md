# Tool force

`if/ToolForceIf.h` defines the force estimate consumed by machine validation.
`toolForceFactory.cc` selects an implementation independently of the machine factory.
`TravelDirectionToolForce` uses only consecutive tool poses and a fixed right-hand spindle
direction. The cutter load has equal components opposite travel and sideways; the returned
workpiece load is its reaction. The sideways direction is the cross product of travel and the tool
axis. The path algorithm checks stock engagement and
submits only cutting moves for force validation.
Only the machine implementation constructs and uses this model; path algorithms receive
machine acceptance without depending on the force representation.
