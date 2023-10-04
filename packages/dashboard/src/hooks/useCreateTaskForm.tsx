import React from 'react';
import { getPlaces } from 'react-components';
import { Subscription } from 'rxjs';
import { RmfIngress } from '../components/rmf-app';

export const useCreateTaskFormData = (rmf: RmfIngress | undefined) => {
  const [waypointNames, setWaypointNames] = React.useState<string[]>([]);
  const [cleaningZoneNames, setCleaningZoneNames] = React.useState<string[]>([]);
  const [pickupPoints, setPickupPoints] = React.useState<Record<string, string>>({});
  const [dropoffPoints, setDropoffPoints] = React.useState<Record<string, string>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const subs: Subscription[] = [];
    subs.push(
      rmf.buildingMapObs.subscribe((map) => {
        const places = getPlaces(map);
        const waypointNames: string[] = [];
        const pickupPoints: Record<string, string> = {};
        const dropoffPoints: Record<string, string> = {};
        const cleaningZoneNames: string[] = [];
        for (const p of places) {
          if (p.pickupHandler !== undefined && p.pickupHandler.length !== 0) {
            pickupPoints[p.vertex.name] = p.pickupHandler;
          }
          if (p.dropoffHandler !== undefined && p.dropoffHandler.length !== 0) {
            dropoffPoints[p.vertex.name] = p.dropoffHandler;
          }
          if (p.cleaningZone !== undefined && p.cleaningZone === true) {
            cleaningZoneNames.push(p.vertex.name);
          }
          waypointNames.push(p.vertex.name);
        }

        setPickupPoints(pickupPoints);
        setDropoffPoints(dropoffPoints);
        setCleaningZoneNames(cleaningZoneNames);
        setWaypointNames(waypointNames);
      }),
    );

    return () => subs.forEach((s) => s.unsubscribe());
  }, [rmf]);

  return { waypointNames, pickupPoints, dropoffPoints, cleaningZoneNames };
};
