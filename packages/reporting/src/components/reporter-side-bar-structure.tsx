import React from 'react';
import SearchIcon from '@mui/icons-material/Search';
import AndroidIcon from '@mui/icons-material/Android';
import ArrowDropUpIcon from '@mui/icons-material/ArrowDropUp';
import KitchenIcon from '@mui/icons-material/Kitchen';
import LocalHospitalIcon from '@mui/icons-material/LocalHospital';
import ArrowForwardIcon from '@mui/icons-material/ArrowForward';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import AccountCircleIcon from '@mui/icons-material/AccountCircle';
import PlaylistAddCheckIcon from '@mui/icons-material/PlaylistAddCheck';

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
      icon: <ArrowForwardIcon />,
      title: 'Dispensers',
      items: [],
      onClick: () => setCurrentReport(Reports.showDispenserStateReport),
    },
    {
      icon: <KitchenIcon />,
      title: 'Doors',
      items: [],
      onClick: () => setCurrentReport(Reports.showDoorStateReport),
    },
    {
      icon: <AndroidIcon />,
      title: 'Fleets',
      items: [],
      onClick: () => setCurrentReport(Reports.showFleetStateReport),
    },
    {
      icon: <LocalHospitalIcon />,
      title: 'Health',
      items: [],
      onClick: () => setCurrentReport(Reports.showHealthReport),
    },
    {
      icon: <ArrowBackIcon />,
      title: 'Ingestor',
      items: [],
      onClick: () => setCurrentReport(Reports.showIngestorStateReport),
    },
    {
      icon: <ArrowDropUpIcon />,
      title: 'Lifts',
      onClick: () => setCurrentReport(Reports.showLiftStateReport),
    },
    {
      icon: <PlaylistAddCheckIcon />,
      title: 'Tasks',
      onClick: () => setCurrentReport(Reports.showTasksReport),
    },
    {
      icon: <AccountCircleIcon />,
      title: 'Users',
      items: [
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
  ] as ExpandableMultilevelMenuProps[];
};
