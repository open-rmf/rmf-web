import React from 'react';
import { Reporter } from '../../components/reports';

export default {
  title: 'Reports',
  component: Reporter,
};

// export const menu = [
//   {
//     icon: <SearchIcon />,
//     title: 'All logs',
//     items: [],
//   },
//   {
//     icon: <BatteryCharging80Icon />,
//     title: 'Charger states',
//   },
//   {
//     icon: <KitchenIcon />,
//     title: 'Doors',
//     items: [],
//   },
//   {
//     icon: <ArrowDropUpIcon />,
//     title: 'Lifts',
//   },
//   {
//     icon: <AndroidIcon />,
//     title: 'Robots',
//     items: [
//       {
//         title: 'Robot states',
//         items: [
//           {
//             title: 'The Dow Theory',
//             to: '/thedowtheory',
//           },
//           {
//             title: 'Charts & Chart Patterns',
//             to: '/chart',
//           },
//           {
//             title: 'Trend & Trend Lines',
//             to: '/trendlines',
//           },
//           {
//             title: 'Support & Resistance',
//             to: '/sandr',
//           },
//         ],
//       },
//       {
//         title: 'Robot Motion Plans',
//         items: [
//           {
//             title: 'The Dow Theory1',
//             to: '/thedowtheory',
//           },
//           {
//             title: 'Charts & Chart Patterns',
//             to: '/chart',
//           },
//           {
//             title: 'Trend & Trend Lines',
//             to: '/trendlines',
//           },
//           {
//             title: 'Support & Resistance',
//             to: '/sandr',
//           },
//         ],
//       },
//       {
//         title: 'Robot Actions',
//         items: [
//           {
//             title: 'The Dow Theory',
//             to: '/thedowtheory',
//           },
//           {
//             title: 'Charts & Chart Patterns',
//             to: '/chart',
//           },
//           {
//             title: 'Trend & Trend Lines',
//             to: '/trendlines',
//           },
//           {
//             title: 'Support & Resistance',
//             to: '/sandr',
//           },
//         ],
//       },
//     ],
//   },
//   {
//     icon: <ListAltIcon />,
//     title: 'Tasks',
//   },
// ];

export const mainView = () => <Reporter />;

// ToStorybook.story = {
//   name: 'to Storybook',
// };
