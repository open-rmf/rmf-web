import { render } from '@testing-library/react';
import React from 'react';

import { LiftTable } from './lift-table';
import { testLifts, testLiftStates } from './test-utils.spec';

describe('lift table', () => {
  it('should render properly', () => {
    const root = render(<LiftTable lifts={testLifts} liftStates={testLiftStates} />);

    // test if the lift names are rendered
    expect(root.getByText('test')).toBeTruthy();
    expect(root.getByText('test1')).toBeTruthy();
    expect(root.getByText('test2')).toBeTruthy();
    expect(root.getByText('test3')).toBeTruthy();
    expect(root.getByText('test4')).toBeTruthy();
    expect(root.getByText('test5')).toBeTruthy();

    // test if op modes are rendered
    expect(root.getByText('AGV')).toBeTruthy();
    expect(root.getByText('Emergency')).toBeTruthy();
    expect(root.getByText('Fire')).toBeTruthy();
    expect(root.getByText('Human')).toBeTruthy();
    expect(root.getByText('Offline')).toBeTruthy();
    expect(root.getByText('Unknown (0)')).toBeTruthy();

    // test if door states are rendered correctly
    expect(root.getAllByText('Closed').length).toEqual(3);
    expect(root.getAllByText('Moving').length).toEqual(1);
    expect(root.getAllByText('Open').length).toEqual(1);
  });
});
