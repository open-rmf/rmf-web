import React from 'react';
import SearchIcon from '@material-ui/icons/Search';
import AndroidIcon from '@material-ui/icons/Android';
import ArrowDropUpIcon from '@material-ui/icons/ArrowDropUp';
import KitchenIcon from '@material-ui/icons/Kitchen';
import BatteryCharging80Icon from '@material-ui/icons/BatteryCharging80';
import ListAltIcon from '@material-ui/icons/ListAlt';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import BusinessCenterIcon from '@material-ui/icons/BusinessCenter';
import { Reports } from './report-list';
import { ExpandableMultilevelMenuProps } from 'react-components';

export type BuildMenuType = (report: Reports) => void;

export const buildReportMenuStructure = (
  setCurrentReport: BuildMenuType,
): ExpandableMultilevelMenuProps[] => {
  return [
    {
      icon: <SearchIcon />,
      title: 'All logs',
      items: [],
      onClick: () => setCurrentReport(Reports.queryAllLogs),
    },
    {
      icon: <BatteryCharging80Icon />,
      title: 'Charger states',
      onClick: () => setCurrentReport(Reports.showChargerStateReport),
    },
    {
      icon: <KitchenIcon />,
      title: 'Doors',
      items: [],
      onClick: () => setCurrentReport(Reports.showDoorStateReport),
    },
    {
      icon: <ArrowDropUpIcon />,
      title: 'Lifts',
      onClick: () => setCurrentReport(Reports.showLiftStateReport),
    },
    {
      icon: <BusinessCenterIcon />,
      title: 'Negotiations',
      onClick: () => setCurrentReport(Reports.showNegotiationsReport),
    },
    {
      icon: <AndroidIcon />,
      title: 'Robots',
      items: [
        {
          title: 'Robot states',
          items: [],
          onClick: () => setCurrentReport(Reports.showRobotStateReport),
        },
        {
          title: 'Robot Motion Plans',
          items: [],
          onClick: () => setCurrentReport(Reports.showRobotMotionPlansReport),
        },
        {
          title: 'Robot Actions',
          items: [],
          onClick: () => setCurrentReport(Reports.showRobotActionReport),
        },
      ],
    },
    {
      icon: <ListAltIcon />,
      title: 'Tasks',
      onClick: () => setCurrentReport(Reports.showTasksReport),
    },
    {
      icon: <AccountCircleIcon />,
      title: 'Users',
      items: [
        {
          title: 'User Actions',
          items: [],
          onClick: () => setCurrentReport(Reports.showUserActionsReport),
        },
        {
          title: 'Logins',
          items: [],
          onClick: () => setCurrentReport(Reports.showLoginsReport),
        },
        {
          title: 'Logouts',
          items: [],
          onClick: () => setCurrentReport(Reports.showLogoutsReport),
        },
        {
          title: 'Login failures',
          items: [],
          onClick: () => setCurrentReport(Reports.showLoginFailuresReport),
        },
      ],
    },
    {
      title: 'Workcell States',
      onClick: () => setCurrentReport(Reports.showWorkCellStatesReport),
    },
  ] as ExpandableMultilevelMenuProps[];
};
