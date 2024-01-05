import type { BuildingMap, GraphNode } from 'api-client';
import { Place } from '../place';
import { RobotData } from './robots-overlay';
import { TrajectoryData } from './trajectories-overlay';
import { RawKnot, RawVelocity, Trajectory } from './trajectory';

export const officeMap: BuildingMap = {
  name: 'building',
  levels: [
    {
      name: 'L1',
      elevation: 0,
      images: [
        {
          name: 'office',
          x_offset: 0,
          y_offset: 0,
          yaw: 0,
          scale: 0.008465494960546494,
          encoding: 'png',
          data: '/assets/office.png',
        },
      ],
      places: [],
      doors: [
        {
          name: 'main_door',
          v1_x: 12.18443775177002,
          v1_y: -2.59969425201416,
          v2_x: 14.079463958740234,
          v2_y: -2.5592799186706543,
          door_type: 6,
          motion_range: 1.5707963705062866,
          motion_direction: 1,
        },
        {
          name: 'coe_door',
          v1_x: 8.256338119506836,
          v1_y: -5.49263334274292,
          v2_x: 7.8990349769592285,
          v2_y: -6.304050922393799,
          door_type: 5,
          motion_range: 1.5707963705062866,
          motion_direction: 1,
        },
        {
          name: 'hardware_door',
          v1_x: 19.447721481323242,
          v1_y: -10.76378345489502,
          v2_x: 19.452404022216797,
          v2_y: -9.874534606933594,
          door_type: 5,
          motion_range: 1.5707963705062866,
          motion_direction: 1,
        },
      ],
      nav_graphs: [
        {
          name: '0',
          vertices: [
            {
              x: 6.897922515869141,
              y: -2.025369644165039,
              name: '',
              params: [
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: false,
                },
              ],
            },
            { x: 10.247854232788086, y: -3.0920557975769043, name: '', params: [] },
            { x: 16.855825424194336, y: -6.881363868713379, name: '', params: [] },
            {
              x: 18.739524841308594,
              y: -6.873981952667236,
              name: '',
              params: [
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: false,
                },
              ],
            },
            {
              x: 16.84633445739746,
              y: -5.404067039489746,
              name: 'pantry',
              params: [
                {
                  name: 'pickup_dispenser',
                  type: 1,
                  value_int: 0,
                  value_float: 0,
                  value_string: 'coke_dispenser',
                  value_bool: false,
                },
                {
                  name: 'is_holding_point',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: false,
                },
              ],
            },
            { x: 13.130566596984863, y: -3.964806079864502, name: '', params: [] },
            { x: 8.91185474395752, y: -6.181352138519287, name: '', params: [] },
            { x: 10.086146354675293, y: -6.97943639755249, name: '', params: [] },
            {
              x: 7.914645195007324,
              y: -7.918869495391846,
              name: '',
              params: [
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: false,
                },
              ],
            },
            { x: 18.8138427734375, y: -11.0789794921875, name: '', params: [] },
            { x: 18.794422149658203, y: -10.372406959533691, name: '', params: [] },
            { x: 7.9883036613464355, y: -10.780257225036621, name: '', params: [] },
            { x: 9.376501083374023, y: -11.142885208129883, name: '', params: [] },
            {
              x: 6.264034271240234,
              y: -3.515686273574829,
              name: 'supplies',
              params: [
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
              ],
            },
            { x: 18.729019165039062, y: -3.895981550216675, name: '', params: [] },
            {
              x: 19.89569854736328,
              y: -3.4071500301361084,
              name: 'lounge',
              params: [
                {
                  name: 'is_holding_point',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: false,
                },
              ],
            },
            {
              x: 10.433053970336914,
              y: -5.5750956535339355,
              name: 'tinyRobot1_charger',
              params: [
                {
                  name: 'is_charger',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
                {
                  name: 'is_holding_point',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
              ],
            },
            { x: 11.565889358520508, y: -6.99680757522583, name: '', params: [] },
            { x: 15.298604965209961, y: -6.92883825302124, name: '', params: [] },
            { x: 11.55336856842041, y: -11.315971374511719, name: '', params: [] },
            { x: 11.573761940002441, y: -9.250288963317871, name: '', params: [] },
            { x: 15.15718936920166, y: -11.227091789245605, name: '', params: [] },
            { x: 15.166704177856445, y: -9.242974281311035, name: '', params: [] },
            { x: 6.517305374145508, y: -5.23292875289917, name: '', params: [] },
            {
              x: 5.346484661102295,
              y: -4.976813793182373,
              name: 'coe',
              params: [
                {
                  name: 'is_holding_point',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: false,
                },
              ],
            },
            { x: 17.067495346069336, y: -11.097883224487305, name: '', params: [] },
            { x: 20.893653869628906, y: -10.30837345123291, name: '', params: [] },
            {
              x: 20.9482479095459,
              y: -7.497346878051758,
              name: 'hardware_2',
              params: [
                {
                  name: 'dropoff_ingestor',
                  type: 1,
                  value_int: 0,
                  value_float: 0,
                  value_string: 'coke_ingestor',
                  value_bool: false,
                },
                {
                  name: 'is_holding_point',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: false,
                },
              ],
            },
            {
              x: 20.42369270324707,
              y: -5.312098026275635,
              name: 'tinyRobot2_charger',
              params: [
                {
                  name: 'is_charger',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
                {
                  name: 'is_holding_point',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
                {
                  name: 'is_parking_spot',
                  type: 4,
                  value_int: 0,
                  value_float: 0,
                  value_string: '',
                  value_bool: true,
                },
              ],
            },
          ],
          edges: [
            { v1_idx: 0, v2_idx: 1, params: [], edge_type: 0 },
            { v1_idx: 2, v2_idx: 3, params: [], edge_type: 0 },
            { v1_idx: 2, v2_idx: 4, params: [], edge_type: 0 },
            { v1_idx: 5, v2_idx: 1, params: [], edge_type: 0 },
            { v1_idx: 1, v2_idx: 6, params: [], edge_type: 0 },
            { v1_idx: 6, v2_idx: 7, params: [], edge_type: 0 },
            { v1_idx: 6, v2_idx: 8, params: [], edge_type: 0 },
            { v1_idx: 9, v2_idx: 10, params: [], edge_type: 0 },
            { v1_idx: 8, v2_idx: 11, params: [], edge_type: 0 },
            { v1_idx: 11, v2_idx: 12, params: [], edge_type: 0 },
            { v1_idx: 0, v2_idx: 13, params: [], edge_type: 0 },
            { v1_idx: 14, v2_idx: 5, params: [], edge_type: 0 },
            { v1_idx: 14, v2_idx: 15, params: [], edge_type: 0 },
            { v1_idx: 7, v2_idx: 16, params: [], edge_type: 0 },
            { v1_idx: 7, v2_idx: 17, params: [], edge_type: 0 },
            { v1_idx: 17, v2_idx: 18, params: [], edge_type: 0 },
            { v1_idx: 18, v2_idx: 2, params: [], edge_type: 0 },
            { v1_idx: 19, v2_idx: 20, params: [], edge_type: 0 },
            { v1_idx: 20, v2_idx: 17, params: [], edge_type: 0 },
            { v1_idx: 21, v2_idx: 22, params: [], edge_type: 0 },
            { v1_idx: 22, v2_idx: 18, params: [], edge_type: 0 },
            { v1_idx: 6, v2_idx: 23, params: [], edge_type: 0 },
            { v1_idx: 23, v2_idx: 24, params: [], edge_type: 0 },
            { v1_idx: 3, v2_idx: 10, params: [], edge_type: 0 },
            { v1_idx: 9, v2_idx: 25, params: [], edge_type: 0 },
            { v1_idx: 10, v2_idx: 26, params: [], edge_type: 0 },
            { v1_idx: 26, v2_idx: 27, params: [], edge_type: 0 },
            { v1_idx: 12, v2_idx: 19, params: [], edge_type: 0 },
            { v1_idx: 19, v2_idx: 21, params: [], edge_type: 0 },
            { v1_idx: 21, v2_idx: 25, params: [], edge_type: 0 },
            { v1_idx: 14, v2_idx: 28, params: [], edge_type: 0 },
            { v1_idx: 14, v2_idx: 3, params: [], edge_type: 0 },
          ],
          params: [],
        },
      ],
      wall_graph: { name: '', vertices: [], edges: [], params: [] },
    },
  ],
  lifts: [],
};

export const officeL1Bounds: L.LatLngBoundsExpression = [
  [0, 0],
  [-14, 25.7],
];

export function makeRobotData(robot: Partial<RobotData> = {}): RobotData {
  return {
    fleet: 'test_fleet',
    name: 'test_robot',
    model: 'test_model',
    footprint: 1,
    color: '#00a000',
    ...robot,
  };
}

export function makeTrajectory(traj?: Partial<Trajectory>): Trajectory {
  return {
    id: 0,
    dimensions: 0.5,
    fleet_name: 'test_fleet',
    robot_name: 'test_robot',
    map_name: 'test_map',
    segments: [
      {
        t: 0,
        v: [0, 0, 0.0],
        x: [10, -10, 0],
      },
      {
        t: 1000,
        v: [10, 0, 0],
        x: [5, -10, 0],
      },
    ],
    shape: 'circle',
    ...traj,
  };
}

export function makeTrajectoryData(trajData: Partial<TrajectoryData> = {}): TrajectoryData {
  return {
    trajectory: makeTrajectory(),
    color: '#00a000',
    conflict: false,
    loopAnimation: true,
    ...trajData,
  };
}

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
 * +ve theta = turn right
 * -ve theta = turn left
 *
 * Direction of the velocity is handled in determineVelocity function
 */

const startingTheta = {
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
 * 3) Staring points for x and y
 * 4) Starting configuration
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
  if (direction === 'up') return [0.0, velocity, theta];
  else if (direction === 'down') return [0.0, -velocity, 0.0];
  else if (direction === 'right') return [velocity, 0.0, theta];
  else return [-velocity, 0.0, 0.0];
};

/**
 * determine the index to retrieve velocity in RawVelocity
 */
const determineVelocityIndex = (direction: string): number => {
  switch (direction) {
    case 'up':
    case 'down':
      return 1;
    case 'left':
    case 'right':
      return 0;
    default:
      return 2;
  }
};

/**
 * turning direction, 0 = right and 1 = left
 */
const determineThetaVelocity = (direction: number, thetaVelocity: number): number => {
  return direction === 0 ? thetaVelocity : -thetaVelocity;
};

/**
 * Calculate next theta
 */
const calculateTheta = (currTheta: number, thetaVelocity: number): number => {
  if (currTheta + thetaVelocity > Math.PI) {
    return -Math.PI + (currTheta + thetaVelocity - Math.PI);
  } else if (currTheta + thetaVelocity < -Math.PI) {
    return Math.PI - (currTheta + thetaVelocity + Math.PI);
  } else {
    return currTheta + thetaVelocity;
  }
};

/**
 * determine the direction of the straight segment after making a turn
 */
const determineDirection = (currDir: string, thetaVelocity: number): string => {
  if (currDir === 'up' && thetaVelocity > 0) return 'right';
  else if (currDir === 'up' && thetaVelocity < 0) return 'left';
  else if (currDir === 'down' && thetaVelocity > 0) return 'right';
  else if (currDir === 'down' && thetaVelocity < 0) return 'left';
  else if (currDir === 'right' && thetaVelocity > 0) return 'up';
  else if (currDir === 'right' && thetaVelocity < 0) return 'down';
  else if (currDir === 'left' && thetaVelocity > 0) return 'up';
  else return 'down';
};

/**
 * 5 < startX < 20
 * -11 < startY < -7
 */
const createSegments = (
  startX: number,
  startY: number,
  startTheta: number,
  direction: string,
): RawKnot[] => {
  // Change the number of turning points, start time and velocity here
  let turningPoints = 2;
  let startTime = 2000;
  const velocity = 0.5;
  const segment: RawKnot[] = [];
  const interval = 500;
  const thetaVelocity = 0.25 * Math.PI;

  let currVelocity = determineVelocity(startTheta, direction, velocity);
  let currVelocityIndex = startTheta > -Math.PI * 0.5 ? 1 : 0;
  let currX = startX;
  let currY = startY;
  let currTheta = startTheta;
  let currDirection = direction;

  while (turningPoints > -1) {
    // generate number of waypoints per turn
    const pointsPerStraightSegment = generateNumber(8, 10);

    // generate points for a straight segment
    for (let i = 0; i < pointsPerStraightSegment; i++) {
      segment.push({ t: startTime, v: currVelocity, x: [currX, currY, currTheta] });

      startTime += interval;
      const distance = currVelocity[currVelocityIndex] * (interval / 1000);
      currX = currVelocityIndex === 0 ? (currX += distance) : currX;
      currY = currVelocityIndex === 1 ? (currY += distance) : currY;
    }

    if (turningPoints > 0) {
      // static point before turning
      startTime += interval;
      const turningDirection = generateNumber(0, 1);
      const currThetaVelocity = determineThetaVelocity(turningDirection, thetaVelocity);
      segment.push({
        t: startTime,
        v: [0.0, 0.0, currThetaVelocity],
        x: [currX, currY, currTheta],
      });

      // 2 points needed to turn at a speed of 1/4 Radian per second
      for (let i = 0; i < 2; i++) {
        const thetaHolder = calculateTheta(currTheta, currThetaVelocity);
        currTheta = thetaHolder;
        startTime += 500;
        segment.push({
          t: startTime,
          v: [0.0, 0.0, currThetaVelocity],
          x: [currX, currY, currTheta],
        });
      }

      // last static point after finishing the turn
      startTime += interval;
      segment.push({ t: startTime, v: [0.0, 0.0, 0.0], x: [currX, currY, currTheta] });

      // prep for next straight segment
      currDirection = determineDirection(currDirection, currThetaVelocity);
      currVelocity = determineVelocity(0.0, currDirection, velocity);
      currVelocityIndex = determineVelocityIndex(currDirection);
      startTime += interval;
    }

    turningPoints -= 1;
  }
  return segment;
};

/**
 * Create a bunch of trajectories
 * numberOfTraj must be >= 1
 */
export const createRandomTrajectories = (numberOfTraj: number): Trajectory[] => {
  const trajHolder = [];

  for (let i = 0; i < numberOfTraj; i++) {
    let knotHolder: RawKnot[] = [];
    const startX = generateNumber(0, 20);
    const startY = generateNumber(0, -20);
    const startConfiguration = generateNumber(1, 4);

    switch (startConfiguration) {
      case 1:
        knotHolder = createSegments(
          startX,
          startY,
          startingTheta.horizontal.value,
          startingTheta.horizontal.direction.right,
        );
        break;
      case 2:
        knotHolder = createSegments(
          startX,
          startY,
          startingTheta.horizontal.value,
          startingTheta.horizontal.direction.left,
        );
        break;
      case 3:
        knotHolder = createSegments(
          startX,
          startY,
          startingTheta.vertical.value,
          startingTheta.vertical.direction.up,
        );
        break;
      case 4:
        knotHolder = createSegments(
          startX,
          startY,
          startingTheta.vertical.value,
          startingTheta.vertical.direction.down,
        );
        break;
    }
    trajHolder.push({
      dimensions: 0.3,
      fleet_name: 'tinyRobot',
      id: i,
      robot_name: 'tinyRobot' + i.toString(),
      segments: knotHolder,
      shape: 'circle',
      map_name: 'L1',
    });
  }
  return trajHolder;
};

export function makeVertex(vertex: Partial<GraphNode> = {}): GraphNode {
  return {
    x: 0,
    y: 0,
    name: 'test_vertex',
    params: [],
    ...vertex,
  };
}

export function makePlace(place: Partial<Place> = {}): Place {
  return {
    level: 'test_level',
    vertex: makeVertex({ name: 'test_place' }),
    ...place,
  };
}
