import { render } from '@testing-library/react';
import React from 'react';
import { LMap } from './map';
import { officeL1Bounds } from './test-utils.spec';
import { WorkcellsOverlay } from './workcells-overlay';

describe('WorkcellsOverlay', () => {
  it('smoke test', () => {
    const root = render(
      <LMap bounds={officeL1Bounds}>
        <WorkcellsOverlay
          bounds={officeL1Bounds}
          workcells={[{ guid: 'test_workcell', location: [0, 0] }]}
        />
      </LMap>,
    );
    expect(() => root.getByLabelText('test_workcell')).not.toThrow();
  });
});
