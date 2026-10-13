import { Configuration } from './utils/InstallerConfiguration';
import background01 from 'renderer/assets/a380x-xp/backgrounds/01.webp';
import background02 from 'renderer/assets/a380x-xp/backgrounds/02.webp';
import background03 from 'renderer/assets/a380x-xp/backgrounds/03.webp';
import background04 from 'renderer/assets/a380x-xp/backgrounds/04.webp';
import background05 from 'renderer/assets/a380x-xp/backgrounds/05.webp';
import background06 from 'renderer/assets/a380x-xp/backgrounds/06.webp';
import background07 from 'renderer/assets/a380x-xp/backgrounds/07.webp';
import background08 from 'renderer/assets/a380x-xp/backgrounds/08.webp';
import background09 from 'renderer/assets/a380x-xp/backgrounds/09.webp';
import background10 from 'renderer/assets/a380x-xp/backgrounds/10.webp';
import background11 from 'renderer/assets/a380x-xp/backgrounds/11.webp';
import background12 from 'renderer/assets/a380x-xp/backgrounds/12.webp';
import background13 from 'renderer/assets/a380x-xp/backgrounds/13.webp';
import background14 from 'renderer/assets/a380x-xp/backgrounds/14.webp';
import background15 from 'renderer/assets/a380x-xp/backgrounds/15.webp';
import amdbBackground from 'renderer/assets/a380x-xp/amdb-background.webp';
import titleDark from 'renderer/assets/a380x-xp/title-dark.svg';
import titleLight from 'renderer/assets/a380x-xp/title-light.svg';
import amdbTitleDark from 'renderer/assets/a380x-xp/amdb-title-dark.svg';
import amdbTitleLight from 'renderer/assets/a380x-xp/amdb-title-light.svg';
import publisherLogo from 'renderer/assets/a380x-xp/publisher.svg';

const REPO_OWNER = 'Vihaan2012-cmyk';
const REPO_NAME = 'Flybywire-A380X-for-Xplane-12';
const REPO_URL = `https://github.com/${REPO_OWNER}/${REPO_NAME}`;
const AMDB_REPO_NAME = 'Free-Airport-Mapping-DB';
const AMDB_URL = `https://github.com/${REPO_OWNER}/${AMDB_REPO_NAME}`;

const xplaneOnly =
  '<span class="text-utility-orange">⚠ Only for X-Plane 12.</span><br/>' +
  '<span class="text-utility-orange">For Microsoft Flight Simulator, use FlyByWire\'s own installer.</span>\n\n';

const updates =
  'Updates come from GitHub: once installed, only what changed since your version is downloaded. ' +
  "SASL is downloaded from 1-sim and X-Plane Sounds' A380 Sound Pack (RR) from its own link; the .acf and " +
  "wing airfoils are built from your own X-Plane 12's Airbus A330-300.\n\n";

export const defaultConfiguration: Configuration = {
  version: 1,
  publishers: [
    {
      name: 'A380X for X-Plane',
      key: 'a380x-xp',
      logoUrl: publisherLogo,
      logoSize: 36,
      defs: [
        {
          kind: 'addonCategory',
          key: 'aircraft',
          title: 'Aircraft',
        },
        {
          kind: 'addonCategory',
          key: 'amdb',
          styles: ['align-bottom'],
        },
      ],
      addons: [
        {
          key: 'a380x-xplane12',
          name: 'A380X',
          simulator: 'xplane12',
          engine: 'aircraft',
          repoOwner: REPO_OWNER,
          repoName: REPO_NAME,
          category: '@aircraft',
          aircraftName: 'A380-842',
          titleImageUrl: titleDark,
          titleImageUrlSelected: titleLight,
          enabled: true,
          backgroundImageUrls: [
            background01,
            background02,
            background03,
            background04,
            background05,
            background06,
            background07,
            background08,
            background09,
            background10,
            background11,
            background12,
            background13,
            background14,
            background15,
          ],
          shortDescription: 'Airbus A380-800',
          description:
            'The A380 is the largest passenger airliner ever built: a full-length double-deck, four-engine ' +
            'wide-body with two hydraulic systems at 5,000 psi backed by electrical actuators, four variable-frequency ' +
            'generators and 22 wheels. The A380-842 flies on four Rolls-Royce Trent 972B-84 engines.\n\n' +
            "This is an unofficial, study-level port of FlyByWire Simulations' A380X to X-Plane 12. FlyByWire's own " +
            'A380 systems, fly-by-wire and instruments run inside X-Plane through a native plugin.',
          techSpecs: [
            { name: 'Engines', value: '4 × RR Trent 972B-84' },
            { name: 'Take-off thrust', value: '341 kN each' },
            { name: 'APU', value: 'PW980A' },
            { name: 'Length', value: '72.72 m' },
            { name: 'Wingspan', value: '79.75 m' },
            { name: 'Height', value: '24.09 m' },
            { name: 'MTOW', value: '575 t' },
            { name: 'MLW', value: '394 t' },
            { name: 'MZFW', value: '369 t' },
            { name: 'Fuel', value: '320,000 L' },
            { name: 'Fuel tanks', value: '11' },
            { name: 'Hydraulics', value: '2 × 5,000 psi' },
            { name: 'Generators', value: '4 VFG + 2 APU' },
            { name: 'Wheels', value: '22' },
            { name: 'Range', value: '8,000 nm' },
            { name: 'Max operating Mach', value: 'M 0.89' },
            { name: 'Ceiling', value: 'FL430' },
          ],
          targetDirectory: 'FlyByWire A380X',
          tracks: [
            {
              name: 'Stable (4K)',
              key: 'a380x-xp-stable-4k',
              url: REPO_URL,
              description:
                xplaneOnly +
                "Includes FlyByWire's 4K cabin, cockpit and exterior textures, with our panel lettering baked in. " +
                'Our recommendation for most systems: sharp panel lettering with 8 GB of graphics card VRAM or more.\n\n' +
                updates,
              isExperimental: false,
              releaseModel: {
                type: 'xplaneEngine',
                textures: 4096,
              },
            },
            {
              name: 'Stable (8K)',
              key: 'a380x-xp-stable-8k',
              url: REPO_URL,
              description:
                xplaneOnly +
                "Includes FlyByWire's 8K full resolution cabin, cockpit and exterior textures. This is the full " +
                'fidelity experience if your system is powerful enough to support it. Realistic and in high detail.\n\n' +
                '* 12 GB of graphics card VRAM or more recommended \n\n' +
                '* A larger download than the 4K version \n\n' +
                updates,
              isExperimental: false,
              releaseModel: {
                type: 'xplaneEngine',
                textures: 8192,
              },
            },
            {
              name: 'Stable (2K)',
              key: 'a380x-xp-stable-2k',
              url: REPO_URL,
              description:
                xplaneOnly +
                'Every cabin, cockpit and exterior texture downscaled to 2K after the download. Choose this option for ' +
                'reduced stutters and better performance. Especially, if you intend to use the following:\n\n' +
                '* Frame generation \n\n' +
                '* Virtual Reality (VR) \n\n' +
                '* or are otherwise limited by your graphics card VRAM amount (8 GB or less). \n\n' +
                'Small panel lettering gets a little softer. Going back to 4K or 8K downloads the aircraft again.\n\n' +
                updates,
              isExperimental: false,
              releaseModel: {
                type: 'xplaneEngine',
                textures: 2048,
              },
            },
          ],
          myInstallPage: {
            links: [
              {
                url: REPO_URL,
                title: 'GitHub',
              },
              {
                url: `${REPO_URL}/issues`,
                title: 'Report a problem',
              },
            ],
            directories: [],
          },
        },
        {
          key: 'amdb-bridge',
          name: 'AMDB Bridge',
          simulator: 'xplane12',
          engine: 'amdb-bridge',
          repoOwner: REPO_OWNER,
          repoName: AMDB_REPO_NAME,
          category: '@amdb',
          aircraftName: 'Airport maps & charts',
          titleImageUrl: amdbTitleDark,
          titleImageUrlSelected: amdbTitleLight,
          enabled: true,
          backgroundImageUrls: [amdbBackground],
          shortDescription: 'AMDB Bridge',
          description:
            "AMDB Bridge serves the A380X's airport moving map (OANS) and the EFB's Charts tab from the Free Airport " +
            'Mapping DB, built from OpenStreetMap data (airport maps © OpenStreetMap contributors, ODbL). The ' +
            'aircraft finds it on this computer by itself; without it, the OANS still loads airports from the ' +
            'dataset directly.',
          targetDirectory: 'AMDB Bridge',
          tracks: [
            {
              name: 'Release',
              key: 'amdb-bridge-release',
              url: AMDB_URL,
              description:
                '<span class="text-utility-orange">⚠ AMDB Bridge installs with its own setup: Install opens its download page.</span>\n\n' +
                'AMDB Bridge is an app that runs next to X-Plane and gives the A380X an airport moving map (OANS) and ' +
                'approach charts on the EFB, from free OpenStreetMap-based airport data. From the taxiways and stands ' +
                'on the ND to charts in the cockpit, it is used for a variety of optional features.',
              isExperimental: false,
              releaseModel: {
                type: 'xplaneEngine',
              },
            },
          ],
          myInstallPage: {
            links: [
              {
                url: `${AMDB_URL}/releases`,
                title: 'Releases',
              },
            ],
            directories: [],
          },
        },
      ],
      buttons: [
        {
          text: 'GitHub',
          action: 'openBrowser',
          url: REPO_URL,
        },
        {
          text: 'Report a problem',
          action: 'openBrowser',
          url: `${REPO_URL}/issues`,
        },
      ],
    },
  ],
};
