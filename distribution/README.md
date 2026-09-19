# Distribution

Free, GPL-3.0. The installer asks the release server for the latest release,
downloads it, verifies its SHA-256 and installs it; running it again updates.

- `worker/`: the release server, a Cloudflare Worker over GitHub Releases of
  Vihaan2012-cmyk/Flybywire-A380X-for-Xplane-12. `GET /manifest` (the latest release's
  manifest.json), `GET /download/<version>/<file>` (a redirect to GitHub's
  download of that asset).
- `../installer/`: the Windows installer (asks for the X-Plane 12 folder and
  the user's SASL 3 archive; SASL is proprietary and never in a release).
- `../tools/package_release.py`: builds a release from the installed aircraft.

## Publishing (one command)

    bash distribution/publish.sh <version> "release notes"

Packages the installed aircraft into parts of at most 250 MB, creates the
GitHub repository the first time (private until you make it public at
release: installers can only download from it once it is public), uploads
the parts and then the manifest to the release `v<version>`, deploys the
Worker (a browser opens once to log in to Cloudflare), and builds
`dist/<version>/FlyByWire-A380X-XP-Installer-<version>.exe` pointing at it.
Run `tools/install.sh` first so the aircraft holds the build you mean to
ship.

## Testing against the private repository

    cd distribution/worker
    gh auth token | npx wrangler secret put GITHUB_TOKEN   # only while private
    npx wrangler dev     # http://127.0.0.1:8787, the installer's default when built without a URL

Delete the secret (`npx wrangler secret delete GITHUB_TOKEN`) once the
repository is public.

## Licences

GPL-3.0 for the code (the source must be offered alongside the binaries:
publish this repository and link it in the release notes). FlyByWire's
original 3D assets are CC BY-NC 4.0 (non-commercial, credit FlyByWire
Simulations). FlyByWire's aircraft was made under Microsoft's Game Content
Usage Rules. Liveries remain their authors'. SASL is never distributed.
