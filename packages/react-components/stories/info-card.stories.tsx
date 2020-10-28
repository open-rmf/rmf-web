import React from 'react';
import { SimpleInfo } from '../lib';

export default {
  title: 'Simple Info',
};

export function SimpleData(): JSX.Element {
  return (
    <SimpleInfo
      data={[
        { name: 'string', value: 'value' },
        { name: 'number', value: 0 },
      ]}
    />
  );
}

export function Array(): JSX.Element {
  return <SimpleInfo data={[{ name: 'strings', value: ['hello', 'world'] }]} />;
}
