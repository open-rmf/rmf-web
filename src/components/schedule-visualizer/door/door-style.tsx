import { makeStyles } from '@material-ui/core';

const doorStyles = makeStyles(() => ({
  doorMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  door: {
    strokeWidth: '0.2',
  },
  doorOpen: {
    stroke: '#AFDDAE',
    strokeDasharray: 0.1,
  },
  doorClose: {
    stroke: '#BC4812',
  },
  doorProcess: {
    stroke: '#E9CE9F',
    strokeDasharray: 0.3,
  },
  doorTransparent: {
    stroke: 'transparent',
  },
}));

export default doorStyles;
