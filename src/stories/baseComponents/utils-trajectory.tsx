import {
  TrajectoryResponse,
  Trajectory,
  RawKnot,
  rawKnotsToKnots,
  RawVelocity,
} from '../../robot-trajectory-manager';
import { bezierControlPoints, knotsToSegmentCoefficientsArray } from '../../util/cublic-spline';

/***
 * Some documentation on the +ve and -ve signs and the directions that they represent
 * before I or anyone get lost reading through code
 *
 * +ve X = right
 * -ve X = left
 *
 * +ve Y = up
 * -ve Y = down
 *
 * +ve theta = turn left
 * -ve theta = turn right
 *
 * Direction of the velocity is handled in determineVelocity function
 */

export const startingTheta = {
  vertical: {
    value: -1.5643726408832297,
    direction: {
      up: 'up',
      down: 'down',
    },
  },
  horizontal: {
    value: -3.1376738367181622,
    direction: {
      left: 'left',
      right: 'right',
    },
  },
};

/***
 * This function is used to generate a random number for
 * various situations stated below:
 *
 * 1) Generating number of waypoints on a straight path per turn
 * 2) Generating a number (0 or 1) to decide turning direction
 */
const generateNumber = (min: number, max: number): number => {
  min = Math.ceil(min);
  max = Math.floor(max + 1);
  return Math.floor(Math.random() * (max - min) + min);
};

/***
 * This function is used to determine RawVelocity at each turn
 */
const determineVelocity = (theta: number, direction: string, velocity: number): RawVelocity => {
  if (Math.floor(theta) === 1 || Math.floor(theta) === -2) {
    if (direction === 'up') return [0.0, velocity, theta];
    else {
      return [0.0, -velocity, 0.0];
    }
  } else {
    if (direction === 'right') return [velocity, 0.0, theta];
    else {
      return [-velocity, 0.0, 0.0];
    }
  }
};

// 5 < startX < 20
// -11 < startY < -1
export const createSegments = (
  startX: number,
  startY: number,
  startTheta: number,
  direction: string,
): RawKnot[] => {
  // fix number for now
  let turningPoints = 1;
  let startTime = 2000;
  let velocity = 0.5;
  const segment: RawKnot[] = [];
  const interval = 500;

  let currVelocity = determineVelocity(startTheta, direction, velocity);
  let currVelocityIndex = startTheta > -Math.PI * 0.5 ? 1 : 0;
  let currX = startX;
  let currY = startY;

  while (turningPoints > 0) {
    // generate number of waypoints per turn
    const pointsPerStraightSegment = generateNumber(10, 12);

    for (let i = 0; i < pointsPerStraightSegment; i++) {
      segment.push({ t: startTime, v: currVelocity, x: [currX, currY, startTheta] });

      startTime += interval;
      const distance = currVelocity[currVelocityIndex] * (interval / 1000);
      currX = currVelocityIndex === 0 ? (currX += distance) : currX;
      currY = currVelocityIndex === 1 ? (currY += distance) : currY;
    }

    // static point before turning
    // startTime += interval;

    turningPoints -= 1;
  }
  console.log(segment);
  return segment;
};
