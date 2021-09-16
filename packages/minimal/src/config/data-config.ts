export interface DataConfig {
  locationDetails: {
    name: string;
  };
  loopTaskDetails: {
    start: string;
    end?: string;
  };
  radioGroup?: {
    waypointValues?: string[];
    formLabel: string;
    radioGroupTitle: string;
  };
}

export const dataConfig: DataConfig = (() => {
  const appUser = process.env.REACT_APP_USER;
  switch (appUser) {
    case 'loading-bay-operator':
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

    case 'mobile-operator':
      return {
        locationDetails: {
          name: 'Loading Bay',
        },
        loopTaskDetails: {
          start: 'supplies',
          end: 'pantry',
        },
      };

    default:
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
  }
})();

export default dataConfig;
