import React from 'react';
import axios from 'axios';
import {
  DefaultReportQueryPayload,
  IngestorStateReport,
  IngestorStateRowsType,
} from 'react-components';
import appConfig from '../../app-config';

const IngestorStateReportConfig = () => {
  const getLogs = async (params: DefaultReportQueryPayload): Promise<IngestorStateRowsType> => {
    try {
      const response = await axios.get(`${appConfig.reportingServerUrl}/report/ingestor_state/`, {
        params: {
          toLogDate: params.toLogDate ? params.toLogDate.format() : null,
          fromLogDate: params.fromLogDate ? params.fromLogDate.format() : null,
          offset: params.offset,
          limit: params.limit,
        },
      });
      return response.data as IngestorStateRowsType;
    } catch (error) {
      console.error(error);
      return [];
    }
  };

  return <IngestorStateReport getLogs={getLogs} />;
};

export default IngestorStateReportConfig;
