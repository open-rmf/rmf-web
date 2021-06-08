import React from 'react';
import { DefaultReportQueryPayload, TaskStateReport, TaskStateRowsType } from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';

const TaskStateReportConfig = () => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<TaskStateRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/task_state/`,
      params,
      authenticator.token,
    )) as TaskStateRowsType;
  };

  return <TaskStateReport getLogs={getLogs} />;
};

export default TaskStateReportConfig;
