import React, { FC, useEffect, useState } from 'react';
import AppLogo from 'renderer/assets/a380x-xp/publisher.svg';
import * as packageInfo from '../../../../package.json';
import { shell } from 'electron';
import { ChangelogModal, useModals } from '../Modal';
import { ThirdPartyLicensesModal } from 'renderer/components/Modal/ThirdPartyLicensesModal';

export const AboutSettings: FC = () => {
  const [logoRotation, setLogoRotation] = useState(0);

  useEffect(() => {
    if (logoRotation / 360 > 5) {
      shell.openExternal('https://www.youtube.com/watch?v=dQw4w9WgXcQ?autoplay=1');

      setLogoRotation(0);
    }
  }, [logoRotation]);

  const handleOpenThirdPartyLicenses = () => {
    showModal(<ThirdPartyLicensesModal />);
  };

  const handleLogoClick = () => {
    setLogoRotation((old) => old + 360);
  };

  const { showModal } = useModals();

  return (
    <div className="flex h-full flex-col justify-center gap-y-8 px-16">
      <div className="flex flex-row items-center gap-x-10">
        <img
          className="cursor-pointer"
          src={AppLogo}
          width={60}
          onClick={handleLogoClick}
          style={{ transform: `rotate(${logoRotation}deg)`, transition: 'transform 200ms linear' }}
        />

        <div className="flex flex-col gap-y-3">
          <span className="font-manrope text-4xl font-bold">A380 for X-Plane 12 (Ported from FBW A380)</span>
          <a
            className="font-manrope text-2xl font-bold text-gray-400 hover:text-gray-600"
            onClick={() => {
              showModal(<ChangelogModal />);
            }}
          >
            v{packageInfo.version}
          </a>
        </div>
      </div>

      <div className="flex flex-col justify-center">
        <span className="text-2xl">
          Based on the FlyByWire Installer, Copyright (c) 2020-2026 FlyByWire Simulations and its contributors
        </span>
        <span className="text-2xl">Licensed under the GNU General Public License Version 3</span>

        <span className="mt-4 text-2xl text-gray-200">
          Ported from FlyByWire Simulations&apos; A380X; FlyByWire&apos;s original 3D models are CC BY-NC 4.0.
          Not affiliated with or endorsed by FlyByWire Simulations, Laminar Research, Microsoft or Airbus.
        </span>

        <a className="mt-4 text-xl text-gray-500 hover:text-gray-600" onClick={handleOpenThirdPartyLicenses}>
          Third party licenses
        </a>
      </div>
    </div>
  );
};
