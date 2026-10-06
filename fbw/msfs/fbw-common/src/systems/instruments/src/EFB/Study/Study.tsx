// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React, { useEffect, useState } from 'react';
import { NavLink, Route } from 'react-router-dom';
import { ExclamationTriangleFill } from 'react-bootstrap-icons';
import { SimpleInput } from '../UtilComponents/Form/SimpleInput/SimpleInput';
import { PageLink, PageRedirect } from '../Utils/routing';
import { Catalogue, loadCatalogue } from './catalogue';
import { FailuresBrowser } from './Pages/FailuresBrowser';
import { ComponentsBrowser } from './Pages/ComponentsBrowser';
import { BreakerPanels } from './Pages/BreakerPanels';
import { MelList } from './Pages/MelList';
import { WeightBalance } from './Pages/WeightBalance';
import { ElectricalDepth } from './Pages/ElectricalDepth';
import { HydraulicDepth } from './Pages/HydraulicDepth';
import { EngineAccessoriesDepth } from './Pages/EngineAccessoriesDepth';
import { DerivedFailures } from './Pages/DerivedFailures';
import { SystemPage } from './Pages/SystemDiagrams';

export const Study = () => {
  const [catalogue, setCatalogue] = useState<Catalogue | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState('');

  useEffect(() => {
    let cancelled = false;
    loadCatalogue()
      .then((loaded) => {
        if (!cancelled) {
          setCatalogue(loaded);
        }
      })
      .catch((loadError: Error) => {
        if (!cancelled) {
          setError(loadError.message);
        }
      });
    return () => {
      cancelled = true;
    };
  }, []);

  if (error !== null) {
    return (
      <>
        <h1 className="font-bold">Study</h1>
        <div className="mt-4 h-content-section-reduced rounded-lg border-2 border-theme-accent p-8">
          <div className="flex flex-row items-center space-x-4">
            <ExclamationTriangleFill size={30} className="text-utility-amber" />
            <div>
              <p className="font-bold">The systems catalogue could not be loaded.</p>
              <p className="mt-1 text-theme-unselected">{error}</p>
            </div>
          </div>
        </div>
      </>
    );
  }

  if (catalogue === null) {
    return (
      <>
        <h1 className="font-bold">Study</h1>
        <div className="mt-4 flex h-content-section-reduced items-center justify-center rounded-lg border-2 border-theme-accent">
          <p className="text-theme-unselected">Loading the systems catalogue…</p>
        </div>
      </>
    );
  }

  const query = searchQuery.trim().toUpperCase();

  const pages: StudyPage[] = [
    { group: 'Failures', name: 'Failures', path: 'failures', render: () => <FailuresBrowser catalogue={catalogue} query={query} /> },
    { group: 'Failures', name: 'Components', path: 'components', render: () => <ComponentsBrowser catalogue={catalogue} query={query} /> },
    { group: 'Failures', name: 'Breakers', path: 'breakers', render: () => <BreakerPanels catalogue={catalogue} query={query} /> },
    { group: 'Failures', name: 'MEL', path: 'mel', render: () => <MelList catalogue={catalogue} query={query} /> },
    { group: 'Failures', name: 'Derived', path: 'derived-failures', render: () => <DerivedFailures /> },
    { group: 'Failures', name: 'Loadsheet', path: 'loadsheet', render: () => <WeightBalance /> },
    { group: 'Systems', name: 'Engine', path: 'engine-depth', render: () => <EngineAccessoriesDepth /> },
    { group: 'Systems', name: 'Electrical', path: 'electrical-depth', render: () => <ElectricalDepth /> },
    { group: 'Systems', name: 'Hydraulic', path: 'hydraulic-depth', render: () => <HydraulicDepth /> },
    { group: 'Systems', name: 'APU', path: 'apu', render: () => <SystemPage titles={['APU']} /> },
    { group: 'Systems', name: 'Bleed', path: 'bleed', render: () => <SystemPage titles={['Bleed System']} /> },
    { group: 'Systems', name: 'Air con', path: 'air-conditioning', render: () => <SystemPage titles={['Air Conditioning']} /> },
    { group: 'Systems', name: 'Press', path: 'pressurisation', render: () => <SystemPage titles={['Cabin Pressurisation']} /> },
    { group: 'Systems', name: 'Gear', path: 'gear', render: () => <SystemPage titles={['Landing Gear and Brakes']} /> },
    { group: 'Systems', name: 'Air data', path: 'air-data', render: () => <SystemPage titles={['Air Data and Inertial']} /> },
    { group: 'Systems', name: 'Fire', path: 'fire', render: () => <SystemPage titles={['Fire Protection']} /> },
    { group: 'Systems', name: 'Radios', path: 'radios', render: () => <SystemPage titles={['Radios']} /> },
  ];
  const groups = [...new Set(pages.map((page) => page.group))];
  const tabs: PageLink[] = [{ name: 'Failures', alias: 'Failures', component: <></> }];

  return (
    <>
      <div className="flex flex-row items-center justify-between space-x-4">
        <h1 className="shrink-0 font-bold">Study</h1>
        <SimpleInput placeholder="Search" className="w-96 uppercase" value={searchQuery} onChange={setSearchQuery} />
        <p className="shrink-0 text-sm text-theme-unselected">
          {catalogue.failures.length.toLocaleString()} failures · {catalogue.components.length.toLocaleString()}{' '}
          components · {catalogue.breakers.length.toLocaleString()} breakers
        </p>
      </div>

      <div className="mt-4 h-content-section-reduced space-y-2 rounded-lg border-2 border-theme-accent p-4">
        {groups.map((group) => (
          <StudyTabs key={group} pages={pages.filter((page) => page.group === group)} />
        ))}

        <div className="pt-2">
          {pages.map((page) => (
            <Route key={page.path} path={`/study/${page.path}`}>
              {page.render()}
            </Route>
          ))}
        </div>
      </div>

      <PageRedirect basePath="/study" tabs={tabs} />
    </>
  );
};

interface StudyPage {
  group: 'Failures' | 'Systems';
  name: string;
  path: string;
  render: () => JSX.Element;
}

const StudyTabs = ({ pages }: { pages: StudyPage[] }) => (
  <nav className="flex">
    <div className="flex divide-x divide-theme-accent overflow-hidden rounded-md border border-theme-accent">
      {pages.map((page) => (
        <NavLink
          key={page.path}
          to={`/study/${page.path}`}
          className="flex items-center px-3 py-1.5 transition duration-300"
          activeClassName="flex items-center px-3 py-1.5 bg-theme-accent"
        >
          {page.name}
        </NavLink>
      ))}
    </div>
  </nav>
);
