import React from 'react';
import axios from 'axios';
import { DefaultReportQueryPayload, FleetStateReport, FleetStateRowsType } from 'react-components';
import appConfig from '../../app-config';

const FleetStateReportConfig = () => {
  const getLogs = async (params: DefaultReportQueryPayload): Promise<FleetStateRowsType> => {
    try {
      // Gets data served by the project itself
      const response = await axios.get(`${appConfig.reportingServerUrl}/report/fleet_state/`, {
        params: {
          toLogDate: params.toLogDate ? params.toLogDate.format() : null,
          fromLogDate: params.fromLogDate ? params.fromLogDate.format() : null,
          offset: params.offset,
          limit: params.limit,
        },
      });
      return response.data as FleetStateRowsType;
    } catch (error) {
      console.error(error);
      return [];
    }
  };

  return <FleetStateReport getLogs={getLogs} />;
};

export default FleetStateReportConfig;
