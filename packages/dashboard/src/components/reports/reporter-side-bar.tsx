import React from 'react';
import SearchIcon from '@material-ui/icons/Search';
import AndroidIcon from '@material-ui/icons/Android';
import ArrowDropUpIcon from '@material-ui/icons/ArrowDropUp';
import KitchenIcon from '@material-ui/icons/Kitchen';
import BatteryCharging80Icon from '@material-ui/icons/BatteryCharging80';
import ListAltIcon from '@material-ui/icons/ListAlt';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import BusinessCenterIcon from '@material-ui/icons/BusinessCenter';

interface BuildMenuProps {
  allLogs: () => void;
  doorsStateReport: () => void;
}

export const buildMenu = (props: BuildMenuProps) => {
  return [
    {
      icon: <SearchIcon />,
      title: 'All logs',
      items: [],
      onClick: props.allLogs,
    },
    {
      icon: <BatteryCharging80Icon />,
      title: 'Charger states',
    },
    {
      icon: <KitchenIcon />,
      title: 'Doors',
      items: [],
      onClick: props.doorsStateReport,
    },
    {
      icon: <ArrowDropUpIcon />,
      title: 'Lifts',
    },
    {
      icon: <BusinessCenterIcon />,
      title: 'Negotiations',
    },
    {
      icon: <AndroidIcon />,
      title: 'Robots',
      items: [
        {
          title: 'Robot states',
          items: [],
        },
        {
          title: 'Robot Motion Plans',
          items: [],
        },
        {
          title: 'Robot Actions',
          items: [],
        },
      ],
    },
    {
      icon: <ListAltIcon />,
      title: 'Tasks',
    },
    {
      icon: <AccountCircleIcon />,
      title: 'Users',
      items: [
        {
          title: 'User Actions',
          items: [],
        },
        {
          title: 'Logins',
          items: [],
        },
        {
          title: 'Logouts',
          items: [],
        },
        {
          title: 'Login failures',
          items: [],
        },
      ],
    },
    {
      title: 'Workcell States',
    },
  ];
};
