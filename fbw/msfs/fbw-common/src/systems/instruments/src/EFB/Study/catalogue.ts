// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

export interface AtaChapter {
  number: number;
  name: string;
}

export interface CatalogueFailure {
  id: number;
  name: string;
  ataChapterNumber: number;
  component: string;
  cause: string;
}

export interface CatalogueComponent {
  id: string;
  name: string;
  ataChapterNumber: number;
  instance: number | null;
  parameters: { name: string; meaning: string; healthyValue: string }[];
  failures: number[];
}

export interface CatalogueAlert {
  key: string;
  ataChapterNumber: number;
  title: string;
  level: string;
  aural: string | null;
  masterLight: string | null;
  confirmSeconds: number | null;
  inhibitedInPhases: number[];
  statusPageLines: string[];
  inoperativeSystemsListEntries: string[];
  failures: number[];
  procedureLineCount: number;
}

export interface CatalogueBreaker {
  id: string;
  name: string;
  ataChapterNumber: number;
  bus: string;
  ratingAmperes: number;
  kind: string;
  consumer: string;
  basis: string;
  panel: string;
  row: number;
  column: number;
  label: string;
  protectsModelledLoad: boolean;
}

export interface CatalogueMelEntry {
  melReference: string;
  ataChapterNumber: number;
  failures: number[];
}

export interface Catalogue {
  schemaVersion: number;
  generatedAtUnixSeconds: number;
  chapters: AtaChapter[];
  failures: CatalogueFailure[];
  components: CatalogueComponent[];
  alerts: CatalogueAlert[];
  breakers: CatalogueBreaker[];
  melEntries: CatalogueMelEntry[];
}

const CATALOGUE_URLS = ['/VFS/html_ui/Pages/VCockpit/Instruments/A380X/EFB/catalogue.json', 'catalogue.json'];

export function fetchFirst(urls: string[]): Promise<Response> {
  const [url, ...rest] = urls;
  return fetch(url).then(
    (response) => (!response.ok && rest.length > 0 ? fetchFirst(rest) : response),
    (error) => {
      if (rest.length > 0) {
        return fetchFirst(rest);
      }
      throw error;
    },
  );
}

let cached: Catalogue | null = null;
let inFlight: Promise<Catalogue> | null = null;

export function loadCatalogue(): Promise<Catalogue> {
  if (cached) {
    return Promise.resolve(cached);
  }
  if (inFlight) {
    return inFlight;
  }

  inFlight = fetchFirst(CATALOGUE_URLS)
    .then((response) => {
      if (!response.ok) {
        throw new Error(`catalogue.json: HTTP ${response.status}`);
      }
      return response.json();
    })
    .then((data: Catalogue) => {
      cached = data;
      inFlight = null;
      return data;
    })
    .catch((error) => {
      inFlight = null;
      throw error;
    });

  return inFlight;
}

export function chapterNames(catalogue: Catalogue): Map<number, string> {
  return new Map(catalogue.chapters.map((chapter) => [chapter.number, chapter.name]));
}

export function byChapter<T extends { ataChapterNumber: number }>(rows: T[]): Map<number, T[]> {
  const grouped = new Map<number, T[]>();
  rows.forEach((row) => {
    const existing = grouped.get(row.ataChapterNumber);
    if (existing) {
      existing.push(row);
    } else {
      grouped.set(row.ataChapterNumber, [row]);
    }
  });
  return new Map([...grouped.entries()].sort(([a], [b]) => a - b));
}

export function panelGrid(breakers: CatalogueBreaker[]): Map<number, CatalogueBreaker[]> {
  const rows = new Map<number, CatalogueBreaker[]>();
  breakers.forEach((breaker) => {
    const existing = rows.get(breaker.row);
    if (existing) {
      existing.push(breaker);
    } else {
      rows.set(breaker.row, [breaker]);
    }
  });
  rows.forEach((row) => row.sort((a, b) => a.column - b.column));
  return new Map([...rows.entries()].sort(([a], [b]) => a - b));
}

export function humanisePanel(panel: string): string {
  return panel.replace(/([A-Z])/g, ' $1').replace(/^./, (c) => c.toUpperCase());
}

export function lvarKey(id: string): string {
  return id.replace(/[^A-Za-z0-9]/g, '_').toUpperCase();
}
