import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import { Circle, Polyline } from 'react-leaflet';

export interface RobotMarkerProps {
  robotState: RomiCore.RobotState;
  footprint: number;
  color: string;
  onclick?: (robot: RomiCore.RobotState) => void;
}

export function RobotMarker(props: RobotMarkerProps): JSX.Element {
  const center = new L.LatLng(props.robotState.location.y, props.robotState.location.x);

  const yaw = props.robotState.location.yaw;
  const nose = new L.LatLng(
    center.lat + Math.sin(yaw) * props.footprint,
    center.lng + Math.cos(yaw) * props.footprint,
  );

  function handleClick(): void {
    props.onclick && props.onclick(props.robotState);
  }

  return (
    <React.Fragment>
      <Circle
        center={center}
        fillOpacity={1}
        radius={props.footprint}
        color={props.color}
        onclick={handleClick}
      />
      <Polyline positions={[center, nose]} color="black" interactive={false} />
    </React.Fragment>
  );
}
