import { Meta, Story } from '@storybook/react';
import React from 'react';
import { customTheme } from '../lib';
import { Typography } from '@material-ui/core';

export default {
  title: 'Theme',
} as Meta;

interface ColorDisplayProps {
  color: string | undefined;
}

const ColorDisplay = (props: ColorDisplayProps): JSX.Element => {
  const { color } = props;
  let colorDesc;

  switch (color) {
    case '#1F396B':
      colorDesc = 'Dark Corn Flower Blue (Main)';
      break;
    case '#20a39e':
      colorDesc = 'Light Sea Green (Secondary)';
      break;
    case '#000000':
      colorDesc = 'Black';
      break;
    case '#6B7D7D':
      colorDesc = 'Xanadu';
      break;
    case '#103375':
      colorDesc = "St Patrick's Blue";
      break;
    case '#5873A8':
      colorDesc = 'Blue Yonder';
      break;
    case '#FBFCFF':
      colorDesc = 'Ghost White';
      break;
    case '#DEA54B':
      colorDesc = 'Indian Yellow';
      break;
    case '#F25F5C':
      colorDesc = 'Fire Opal';
      break;
  }

  return (
    <div style={{ margin: '0 1rem' }}>
      <div
        style={{ width: '100px', height: '100px', backgroundColor: color, margin: '0 auto' }}
      ></div>
      <Typography variant="body1" align="center" style={{ width: '100px' }}>
        {colorDesc}
      </Typography>
    </div>
  );
};

export const ThemeStory: Story = (args) => {
  return (
    <div {...args} style={{ display: 'flex' }}>
      <ColorDisplay color={customTheme.palette.primary.main} />
      <ColorDisplay color={customTheme.palette.secondary.main} />
      <ColorDisplay color={customTheme.colors.black} />
      <ColorDisplay color={customTheme.colors.xanadu} />
      <ColorDisplay color={customTheme.colors.stPatricksBlue} />
      <ColorDisplay color={customTheme.colors.blueYonder} />
      <ColorDisplay color={customTheme.colors.ghostWhite} />
      <ColorDisplay color={customTheme.colors.indianYellow} />
      <ColorDisplay color={customTheme.colors.fireOpal} />
    </div>
  );
};
