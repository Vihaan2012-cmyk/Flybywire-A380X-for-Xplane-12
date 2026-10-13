//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { Coordinates } from 'msfs-geo';
import {
  AirportCommunication,
  Airway,
  DatabaseIdent,
  Fix,
  IlsNavaid,
  iso8601CalendarDate,
  NdbNavaid,
  ProcedureLeg,
  VhfNavaid,
  Waypoint,
} from '../../../shared';
import { Airport } from '../../../shared/types/Airport';
import { Approach } from '../../../shared/types/Approach';
import { Arrival } from '../../../shared/types/Arrival';
import { Departure } from '../../../shared/types/Departure';
import { Runway } from '../../../shared/types/Runway';
import { DataInterface } from '../../../shared/DataInterface';
import { Marker } from '../../../shared/types/Marker';
import { IcaoSearchFilter, JS_FacilityAirport, VorType } from './FsTypes';
import { FacilityCache, LoadType } from './FacilityCache';
import { MsfsMapping } from './Mapping';
import { Gate } from '../../../shared/types/Gate';
import { NearbyFacilityType, NearbyFacilityMonitor } from '../../NearbyFacilityMonitor';
import { MsfsNearbyFacilityMonitor } from './MsfsNearbyFacilityMonitor';
import { ErrorLogger } from '../../../shared/types/ErrorLogger';

const DATE_RANGE = /([A-Z]{3})(\d{1,2})([A-Z]{3})(\d{1,2})\/(\d{2})/;
const DAY_MS = 24 * 60 * 60 * 1000;
const CYCLE_DAYS = 28;

function uniqueByDatabaseId<T extends { databaseId: string }>(items: T[]): T[] {
  const seen = new Set<string>();
  return items.filter((item) => {
    if (seen.has(item.databaseId)) {
      return false;
    }
    seen.add(item.databaseId);
    return true;
  });
}

export class MsfsBackend implements DataInterface {
  private readonly cache: FacilityCache;

  private readonly mapping: MsfsMapping;

  constructor(private readonly logError: ErrorLogger) {
    this.cache = new FacilityCache(logError);
    this.mapping = new MsfsMapping(this.cache, logError);
  }

  public async getDatabaseIdent(): Promise<DatabaseIdent> {
    const facilitiesDateRange = SimVar.GetGameVarValue('FLIGHT NAVDATA DATE RANGE', 'string');
    const match = facilitiesDateRange.match(DATE_RANGE);
    if (!match) {
      this.logError(`[MsfsBackend]: Failed to parse facilitiesDateRange: ${facilitiesDateRange}`);
      return { provider: 'Msfs', airacCycle: '', effectiveFrom: '', effectiveTo: '' };
    }
    const [, fromMonth, fromDay, toMonth, toDay, year] = match;
    const from = new Date(`${fromMonth} ${fromDay} ${year}`);
    const to = new Date(`${toMonth} ${toDay} ${year}`);
    if (!(to >= from)) {
      from.setFullYear(from.getFullYear() - 1);
    }
    const end = new Date(from.getFullYear(), from.getMonth(), from.getDate() + CYCLE_DAYS);
    const dayOfYear = Math.round((from.getTime() - new Date(from.getFullYear(), 0, 1).getTime()) / DAY_MS) + 1;
    const cycle = Math.ceil(dayOfYear / CYCLE_DAYS);
    return {
      provider: 'Msfs',
      airacCycle: `${(from.getFullYear() % 100).toString().padStart(2, '0')}${cycle.toString().padStart(2, '0')}`,
      effectiveFrom: iso8601CalendarDate(from.getFullYear(), from.getMonth() + 1, from.getDate()),
      effectiveTo: iso8601CalendarDate(end.getFullYear(), end.getMonth() + 1, end.getDate()),
    };
  }

  public createNearbyFacilityMonitor(type: NearbyFacilityType): NearbyFacilityMonitor {
    return new MsfsNearbyFacilityMonitor(type);
  }

  public async getAirports(idents: string[]): Promise<Airport[]> {
    const airports = await this.cache.getFacilities(
      idents.map((ident) => `A      ${ident}`),
      LoadType.Airport,
    );
    return [...airports.values()].map((airport) => this.mapping.mapAirport(airport));
  }

  public async getDepartures(airportIdentifier: string): Promise<Departure[]> {
    const airport = await this.fetchMsfsAirport(airportIdentifier);
    return airport ? this.mapping.mapDepartures(airport) : [];
  }

  public async getArrivals(airportIdentifier: string): Promise<Arrival[]> {
    const airport = await this.fetchMsfsAirport(airportIdentifier);
    return airport ? this.mapping.mapArrivals(airport) : [];
  }

  public async getApproaches(airportIdentifier: string): Promise<Approach[]> {
    const airport = await this.fetchMsfsAirport(airportIdentifier);
    return airport ? this.mapping.mapApproaches(airport) : [];
  }

  public async getGates(airportIdentifier: string): Promise<Gate[]> {
    const airport = await this.fetchMsfsAirport(airportIdentifier);
    return airport ? this.mapping.mapGates(airport) : [];
  }

  public async getHolds(airportIdentifier: string): Promise<ProcedureLeg[]> {
    const airport = await this.fetchMsfsAirport(airportIdentifier);
    return airport ? this.mapping.mapHolds(airport) : [];
  }

  public async getRunways(airportIdentifier: string): Promise<Runway[]> {
    const airport = await this.fetchMsfsAirport(airportIdentifier);
    return airport ? this.mapping.mapAirportRunways(airport) : [];
  }

  public async getWaypointsAtAirport(airportIdentifier: string): Promise<Waypoint[]> {
    const airport = await this.fetchMsfsAirport(airportIdentifier);
    return airport ? this.mapping.mapAirportWaypoints(airport) : [];
  }

  public async getNdbsAtAirport(_airportIdentifier: string): Promise<NdbNavaid[]> {
    return [];
  }

  public async getIlsAtAirport(airportIdentifier: string, ident?: string, lsIcaoCode?: string): Promise<IlsNavaid[]> {
    const airport = await this.fetchMsfsAirport(airportIdentifier);
    return airport ? this.mapping.mapAirportIls(airport, ident, lsIcaoCode) : [];
  }

  public async getCommunicationsAtAirport(airportIdentifier: string): Promise<AirportCommunication[]> {
    const airport = await this.fetchMsfsAirport(airportIdentifier);
    return airport ? this.mapping.mapAirportCommunications(airport) : [];
  }

  public async getLsMarkers(
    _airportIdentifier: string,
    _runwayIdentifier: string,
    _lsIdentifier: string,
  ): Promise<Marker[]> {
    return [];
  }

  public async getWaypoints(
    idents: string[],
    _ppos?: Coordinates,
    _icaoCode?: string,
    _airportIdent?: string,
  ): Promise<Waypoint[]> {
    const results: Waypoint[] = [];
    for (const ident of idents) {
      const facilities = await this.cache.searchByIdent(ident, IcaoSearchFilter.Intersections, 100);
      results.push(...facilities.map((facility) => this.mapping.mapFacilityToWaypoint(facility)));
    }
    return uniqueByDatabaseId(results);
  }

  public async getNdbNavaids(
    idents: string[],
    _ppos?: Coordinates,
    _icaoCode?: string,
    _airportIdent?: string,
  ): Promise<NdbNavaid[]> {
    const results: NdbNavaid[] = [];
    for (const ident of idents) {
      const facilities = await this.cache.searchByIdent(ident, IcaoSearchFilter.Ndbs, 100);
      results.push(...facilities.map((facility) => this.mapping.mapFacilityToWaypoint(facility)));
    }
    return uniqueByDatabaseId(results);
  }

  public async getVhfNavaids(
    idents: string[],
    _ppos?: Coordinates,
    _icaoCode?: string,
    _airportIdent?: string,
  ): Promise<VhfNavaid[]> {
    const results: VhfNavaid[] = [];
    for (const ident of idents) {
      const facilities = await this.cache.searchByIdent(ident, IcaoSearchFilter.Vors, 100);
      results.push(...facilities.map((facility) => this.mapping.mapFacilityToWaypoint(facility)));
    }
    return uniqueByDatabaseId(results);
  }

  public async getIlsNavaids(
    idents: string[],
    _ppos?: Coordinates,
    _icaoCode?: string,
    _airportIdent?: string,
  ): Promise<IlsNavaid[]> {
    const results: IlsNavaid[] = [];
    for (const ident of idents) {
      const vors = await this.cache.searchByIdent(ident, IcaoSearchFilter.Vors, 100);
      await Promise.all(
        vors
          .filter((vor) => vor.type === VorType.ILS)
          .map(async (vor) => {
            const airport = await this.fetchMsfsAirport(vor.icao.substring(3, 7).trim());
            if (airport) {
              results.push(await this.mapping.mapVorIls(airport, vor));
            }
          }),
      );
    }
    return uniqueByDatabaseId(results);
  }

  public async getFixes(idents: string[], ppos?: Coordinates, icaoCode?: string, airportIdent?: string): Promise<Fix[]> {
    return [
      ...(await this.getWaypoints(idents, ppos, icaoCode, airportIdent)),
      ...(await this.getNdbNavaids(idents, ppos, icaoCode, airportIdent)),
      ...(await this.getVhfNavaids(idents, ppos, icaoCode, airportIdent)),
    ];
  }

  public async getAirways(_idents: string[]): Promise<Airway[]> {
    return [];
  }

  public async getAirwayByFix(ident: string, icaoCode: string, airwayIdent: string): Promise<Airway[]> {
    return this.mapping.getAirway(ident, icaoCode, airwayIdent);
  }

  public async getVhfNavaidFromId(databaseId: string): Promise<VhfNavaid> {
    const vor = await this.cache.getFacility(databaseId, LoadType.Vor);
    if (!vor) {
      throw new Error(`Could not fetch facility "${databaseId}"!`);
    }
    return this.mapping.mapFacilityToWaypoint(vor);
  }

  private async fetchMsfsAirport(ident: string): Promise<JS_FacilityAirport | undefined> {
    return this.cache.getFacility(`A      ${ident}`, LoadType.Airport);
  }
}
