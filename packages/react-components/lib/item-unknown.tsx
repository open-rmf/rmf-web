import React from 'react';
import { SimpleInfo } from './simple-info';

interface Location {
  x: number;
  y: number;
  yaw: number;
  level_name: string;
}

export interface ItemUnknownProps {
  name: string;
  location: Location | undefined;
  errorMsg?: string;
}

export const ItemUnknown = (props: ItemUnknownProps): JSX.Element => {
  const { name, location, errorMsg } = props;

  const data = [
    { name: 'Name', value: name },
    {
      name: 'Location',
      value: location
        ? `${location.level_name} (${location.x.toFixed(3)}, ${location.y.toFixed(3)})`
        : 'Unable to locate',
    },
    { name: 'Error', value: errorMsg ? errorMsg : 'No error message provided' },
  ];

  return <SimpleInfo infoData={data} />;
};
