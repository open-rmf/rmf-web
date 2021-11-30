import React from 'react';
import { DataGrid, GridRenderCellParams } from '@mui/x-data-grid';
import { Typography } from '@mui/material';
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
  const { rows, addMoreRows } = props;

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <DataGrid
        autoHeight={true}
        getRowId={(r) => r.robot.id}
        columns={[
          {
            headerName: 'Fleet',
            field: 'fleet_name',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.fleet.name}</Typography>;
            },
          },
          {
            headerName: 'Robot',
            field: 'robot_name',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.robot.name}</Typography>;
            },
          },
          {
            headerName: 'Battery',
            field: 'robot_battery_percent',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.robot_battery_percent}</Typography>;
            },
          },
          {
            headerName: 'Mode',
            field: 'robot_mode',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.robot_mode}</Typography>;
            },
          },
          {
            headerName: 'Model',
            field: 'robot_model',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.robot.model}</Typography>;
            },
          },
          {
            headerName: 'TaskID',
            field: 'robot_task_id',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.robot_task_id}</Typography>;
            },
          },
          {
            headerName: 'Timestamp',
            field: 'created',
            type: 'datetime',
            filterable: false,
            align: 'center',
            renderCell: (rowData: GridRenderCellParams) => {
              return (
                <Typography data-testid={'fleet-table-date'}>
                  {format(new Date(rowData.value as number), 'MMM dd yyyy hh:mm aaa')}
                </Typography>
              );
            },
          },
        ]}
        rows={rows}
        pageSize={100}
        rowsPerPageOptions={[50, 100]}
        onPageChange={() => {
          if (addMoreRows) {
            addMoreRows();
          }
        }}
        disableColumnMenu={true}
      />
    </div>
  );
};
