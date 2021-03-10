import React from 'react';
import { ReportContainer } from './report-list';

interface ReporterBodyProps {
  currentReport: string;
}

export const ReporterBody = (props: ReporterBodyProps) => {
  const { currentReport } = props;

  const report = React.useMemo(() => {
    return ReportContainer[currentReport];
  }, [currentReport]);

  return <>{report}</>;
};
