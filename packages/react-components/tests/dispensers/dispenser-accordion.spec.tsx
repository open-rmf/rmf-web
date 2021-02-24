import { render } from '@testing-library/react';
import React from 'react';
import { DispenserAccordion } from '../../lib';
import { makeDispenserState } from './test-utils';

it('smoke test', () => {
  render(<DispenserAccordion dispenser={'dispenser'} dispenserState={makeDispenserState()} />);
});
