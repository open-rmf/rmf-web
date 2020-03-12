import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import { Circle, Polyline } from 'react-leaflet';

export interface RobotMarkerProps {
  robotState: RomiCore.RobotState;
  footprint: number;
  color: string;
}

export function RobotMarker(props: RobotMarkerProps): JSX.Element {
  const center = new L.LatLng(props.robotState.location.y, props.robotState.location.x);

  const yaw = props.robotState.location.yaw;
  const nose = new L.LatLng(
    center.lat + Math.sin(yaw) * props.footprint,
    center.lng + Math.cos(yaw) * props.footprint,
  );

  return (
    <React.Fragment>
      <Circle center={center} fillOpacity={1} radius={props.footprint} color={props.color} />
      <Polyline positions={[center, nose]} color="black" />
    </React.Fragment>
  );
}
