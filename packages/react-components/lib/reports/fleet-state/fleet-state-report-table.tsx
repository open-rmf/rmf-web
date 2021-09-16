import React from 'react';
// import MaterialTable from 'material-table';
import { DataGrid } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
// import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';

export type FleetStateRowsType = {
  created: string; //date
  fleet: { id: number; name: string };
  robot: { id: number; name: string; model?: string };
  robot_battery_percent: string;
  robot_location: string;
  robot_mode: string;
  robot_seq: number;
  robot_task_id: string;
}[];

export interface FleetStateReportTable extends DefaultLogTableProps {
  rows: FleetStateRowsType | [];
}

export const FleetStateReportTable = (props: FleetStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;

  return (
    <div style={{ height: tableSize, width: '100%' }}>
      <DataGrid
        getRowId={(r) => r.robot.id}
        columns={[
          {
            headerName: 'Fleet',
            field: 'fleet_name',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.fleet.name}</Typography>;
            },
          },
          {
            headerName: 'Robot',
            field: 'robot_name',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.robot.name}</Typography>;
            },
          },
          {
            headerName: 'Battery',
            field: 'robot_battery_percent',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.robot_battery_percent}</Typography>;
            },
          },
          {
            headerName: 'Mode',
            field: 'robot_mode',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.robot_mode}</Typography>;
            },
          },
          {
            headerName: 'Model',
            field: 'robot_model',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.robot.model}</Typography>;
            },
          },
          {
            headerName: 'TaskID',
            field: 'robot_task_id',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.robot_task_id}</Typography>;
            },
          },
          {
            headerName: 'Timestamp',
            field: 'timestamp',
            type: 'datetime',
            filterable: false,
            align: 'center',
            valueFormatter: (rowData) => {
              return (
                <Typography data-testid={'fleet-table-date'}>
                  {format(new Date(rowData.row.created), 'MMM dd yyyy hh:mm aaa')}
                </Typography>
              );
            },
          },
        ]}
        rows={rows}
        pageSize={100}
        rowsPerPageOptions={[50, 100, 200]}
        onPageChange={(page, pageSize) => {
          if (addMoreRows) {
            rows.length / pageSize - 1 === page && addMoreRows();
          }
        }}
      />
    </div>
  );
};
