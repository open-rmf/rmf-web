import { Meta, Story } from '@storybook/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { DoorCell, DoorCellProps } from './door-cell';
import { makeDoor } from './test-utils.spec';

export default {
  title: 'Door Cell',
} as Meta;

export const Default: Story<Pick<DoorCellProps, 'doorMode'>> = (args) => {
  return <DoorCell door={{ door: makeDoor({ name: 'example_door' }), level: 'L1' }} {...args} />;
};

Default.storyName = 'Door Cell';

Default.args = {
  doorMode: RmfModels.DoorMode.MODE_OPEN,
};
