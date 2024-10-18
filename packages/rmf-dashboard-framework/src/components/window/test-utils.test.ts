import { WindowLayout } from './window-container';

export const makeLayout: () => WindowLayout[] = () => [
  { i: '0', x: 0, y: 0, w: 2, h: 4, minW: 2, minH: 4 },
  { i: '1', x: 2, y: 0, w: 2, h: 4, minW: 2, minH: 4 },
  { i: '2', x: 0, y: 4, w: 2, h: 4, minW: 2, minH: 4 },
  { i: '3', x: 2, y: 4, w: 2, h: 4, minW: 2, minH: 4 },
];
