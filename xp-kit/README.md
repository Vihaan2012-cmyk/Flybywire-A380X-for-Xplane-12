# xp-kit

A small avionics UI library and a build-time transform that run FlyByWire's A380X instruments in X-Plane through the FS2XP converter.

FlyByWire's instrument code is left untouched in its own repository. When the instruments are bundled, the transform moves that code onto this library, so the bundles depend only on xp-kit.

## What's inside

| Path | What it is |
|---|---|
| `src/` | The library: values and derived values (`cell`, `derive`, `feed`, `list`), the event hub, components (`Widget`, `Kit.h` JSX, `Ref`), units, number and time formatting, geo and aviation maths, nav data access. |
| `tools/transform.cjs` | An esbuild plugin that rewrites the instruments' imports and calls onto this library while bundling. |
| `names.json` | The table the transform uses to map names. |

## Build

```sh
npm install
npm run build        # dist/index.js
npm run typecheck
```

## Use in a FlyByWire build

Add the plugin to the esbuild plugin list of the build you bundle with, then point `XP_KIT` at the built library:

```js
const kit = process.env.XP_KIT
  ? require(path.resolve(process.env.XP_KIT, '..', 'tools', 'transform.cjs')).plugin()
  : { name: 'xp-kit-off', setup() {} };
```

```sh
XP_KIT=/path/to/xp-kit/dist <your usual instrument build>
```

The plugin:

- rewrites each instrument's imports to `xp-kit` and resolves that to `$XP_KIT/index.js`;
- strips comments from the bundles;
- fails the build if any import it can't map is left;
- prints a list of anything it didn't recognise.

## Checking a build

Bundles built with and without the transform should behave identically. We check this by running both offline with the same seeded inputs and comparing every screen's frames and every variable write.

## Licence

GPL-3.0-or-later, see `LICENSE`. The instruments it runs are FlyByWire Simulations' work, under their own GPL-3.0 licence.
