import React from 'react';
import axios from 'axios';
import { DefaultReportQueryPayload, DoorStateReport, DoorStateRowsType } from 'react-components';
import appConfig from '../../app-config';

const DoorStateReportConfig = () => {
  const getLogs = async (params: DefaultReportQueryPayload): Promise<DoorStateRowsType> => {
    try {
      // Gets data served by the project itself
      const response = await axios.get(`${appConfig.reportingServerUrl}/report/door_state/`, {
        params: {
          toLogDate: params.toLogDate ? params.toLogDate.format() : null,
          fromLogDate: params.fromLogDate ? params.fromLogDate.format() : null,
          offset: params.offset,
          limit: params.limit,
        },
      });
      return response.data as DoorStateRowsType;
    } catch (error) {
      console.error(error);
      return [];
    }
  };

  return <DoorStateReport getLogs={getLogs} />;
};

export default DoorStateReportConfig;
