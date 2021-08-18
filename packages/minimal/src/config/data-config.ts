export interface DataConfig {
  locationDetails: {
    name: string;
  };
  loopTaskDetails: {
    start: string;
  };
  radioGroup: {
    waypointValues?: string[];
    formLabel: string;
    radioGroupTitle: string;
  };
}

export const dataConfig: DataConfig = (() => {
  const appUser = process.env.REACT_APP_USER;

  if (appUser === 'loading-bay-operator') {
    return {
      locationDetails: {
        name: 'Loading Bay',
      },
      loopTaskDetails: {
        start: 'supplies',
      },
      radioGroup: {
        formLabel: 'Destination',
        radioGroupTitle: 'Destination Values',
      },
    };
  }
  return {
    locationDetails: {
      name: 'Office',
    },
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
