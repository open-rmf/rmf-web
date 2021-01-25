import { StatusIndicator } from 'react-components';

export const makeStatusData = (): StatusIndicator => {
  return {
    doors: {
      door1: { state: true },
      door2: { state: true },
      door3: { state: true },
    },
    lifts: {
      lift1: { state: true },
      lift2: { state: true },
      lift3: { state: true },
      lift4: { state: true },
    },
    robots: {
      robot1: { state: true },
      robot2: { state: true },
      robot3: { state: true },
      robot4: { state: true },
    },
    dispensers: {
      dispenser1: { state: true },
      dispenser2: { state: true },
      dispenser3: { state: true },
      dispenser4: { state: true },
    },
  };
};
