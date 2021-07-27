export interface DataConfig {
  loopTaskDetails: {
    start: string;
  };
  radioGroup: {
    waypointValues: string[];
    formLabel: string;
    radioGroupTitle: string;
  };
}

export const dataConfig: DataConfig = (() => {
  const appUser = process.env.REACT_APP_USER;
  if (!appUser) {
    throw new Error('missing REACT_APP_USER');
  }
  if (appUser === 'admin') {
    return {
      loopTaskDetails: {
        start: 'supplies',
      },
      radioGroup: {
        waypointValues: ['lounge', 'coe', 'hardware', 'pantry'],
        formLabel: 'Destination',
        radioGroupTitle: 'Destination Values',
      },
    };
  }
  return {
    loopTaskDetails: {
      start: 'pantry',
    },
    radioGroup: {
      waypointValues: ['lounge', 'coe', 'hardware'],
      formLabel: 'Destination',
      radioGroupTitle: 'Destination Values',
    },
  };
})();

export default dataConfig;
