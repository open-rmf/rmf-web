import React from 'react';
import { SimpleInfo } from '..';

export default {
  title: 'Simple Info',
};

export function SimpleData() {
  return <SimpleInfo data={{ string: 'string', number: 0 }} />;
}

export function Array() {
  return <SimpleInfo data={{ strings: ['hello', 'world'] }} />;
}
