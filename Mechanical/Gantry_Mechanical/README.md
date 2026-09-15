# Gantry mechanical source archives

These archives were copied without modification on 2026-09-14 from the local
project staging folder:

```text
C:\Users\oswak\Downloads\Gantry_Mechanical
```

They are retained as source/reference material for the mechanical gantry. Keep
the archives intact so their included documentation and license files remain
associated with the models.

## Inventory

| Archive | Size (bytes) | Contents |
| --- | ---: | --- |
| `Bearing.zip` | 8,946 | Four STEP bearing-ring models |
| `Ender 3 Cable Chain, no screws, no unplug cable - 3769941.zip` | 37,333,163 | STL models, example printer G-code, images, README, and license |
| `OpenBuilds V-Slot™ Linear Actuators - 69004.zip` | 10,980,258 | STL models, SketchUp archive, videos, images, README, and license |

## SHA-256

```text
CAE7529FF7CB6AD6EE9F38342CB6ACBF9A29103679AFB2D44184CCF375C05788  Bearing.zip
5A4FB7F57B66F34E2335126229D11A3E3F31087A7461ACAC9656E7646BA6F5CC  Ender 3 Cable Chain, no screws, no unplug cable - 3769941.zip
3998BE7D92216873D7F42F55A696A5199EF6B55E0BB58AB76DEFDA88F2F09FDB  OpenBuilds V-Slot™ Linear Actuators - 69004.zip
```

The hashes above matched the source downloads when copied.

## Safety and reuse notes

- The cable-chain archive contains `G1.gcode`, `G2.gcode`, and `G3.gcode` for a
  different machine. Do not send them to this FluidNC gantry without reviewing
  coordinates, units, feeds, tool commands, and machine limits.
- Review each archive's bundled `LICENSE.txt` before redistributing or modifying
  its content. `Bearing.zip` does not contain an embedded license file, so its
  reuse terms must be established separately.
- The three top-level V-Slot STL files already present in `Mechanical/` also
  occur inside the OpenBuilds archive; they are intentionally not overwritten.
