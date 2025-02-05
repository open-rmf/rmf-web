//==============================================================================
// Basic hello world demo

import { Box, Typography } from '@mui/material';

export const Demo = () => {
  return (
    <Box
      sx={{
        position: 'absolute',
        top: '50%',
        left: '50%',
        transform: 'translate(-50%, -50%)',
        justifyContent: 'center',
        alignItems: 'center',
      }}
    >
      <Typography variant="h6">Hello world!</Typography>
    </Box>
  );
};

export default Demo;

//==============================================================================
// With events

// import React from 'react';
// import { Box, Typography } from '@mui/material';
// import { AppEvents } from '../app-events';

// export const Demo = () => {
//   const [lastClicked, setLastClicked] = React.useState('Nothing clicked so far');

//   AppEvents.zoomIn.subscribe(() => setLastClicked('Zoom in clicked'));
//   AppEvents.zoomOut.subscribe(() => setLastClicked('Zoom out clicked'));

//   return (
//     <Box
//       sx={{
//         position: 'absolute',
//         top: '50%',
//         left: '50%',
//         transform: 'translate(-50%, -50%)',
//         justifyContent: 'center',
//         alignItems: 'center',
//       }}
//     >
//       <Typography variant="h6">
//         {lastClicked}
//       </Typography>
//     </Box>
//   );
// };

// export default Demo;

//==============================================================================
// With table

// import { TableContainer } from '@mui/material';
// import {
//   DataGrid,
//   GridColDef,
//   GridValueGetterParams,
// } from '@mui/x-data-grid';
// import React from 'react';
// import { AppEvents } from '../app-events';

// interface MapAction {
//   index: number;
//   unixMillis: number;
//   action: string;
// }

// export const Demo = () => {
//   const [mapActions, setMapActions] = React.useState<Array<MapAction>>([]);

//   AppEvents.zoomIn.subscribe(() => {
//     setMapActions((prev) => {
//       return [
//         ...prev,
//         {
//           index: prev.length,
//           unixMillis: new Date().valueOf(),
//           action: 'Zoom In',
//         }
//       ];
//     })
//   });
//   AppEvents.zoomOut.subscribe(() => {
//     setMapActions((prev) => {
//       return [
//         ...prev,
//         {
//           index: prev.length,
//           unixMillis: new Date().valueOf(),
//           action: 'Zoom Out',
//         }
//       ];
//     })
//   });

//   const columns: GridColDef[] = [
//     {
//       field: 'unixMillis',
//       headerName: 'Unix millisecond',
//       editable: false,
//       valueGetter: (params: GridValueGetterParams) => params.row.unixMillis,
//       flex: 1,
//       filterable: true,
//       sortable: true,
//     },
//     {
//       field: 'action',
//       headerName: 'Action',
//       editable: false,
//       valueGetter: (params: GridValueGetterParams) => params.row.action,
//       flex: 1,
//       filterable: true,
//       sortable: true,
//     },
//   ];

//   return (
//     <TableContainer sx={{ height: '100% ' }}>
//       <DataGrid
//         getRowId={(l) => l.index}
//         rows={mapActions}
//         pageSize={5}
//         rowHeight={38}
//         columns={columns}
//         rowsPerPageOptions={[5]}
//         density={'standard'}
//         localeText={{
//           noRowsLabel: 'Nothing clicked',
//         }}
//         initialState={{
//           sorting: {
//             sortModel: [{ field: 'doorName', sort: 'asc' }],
//           },
//         }}
//         disableVirtualization={true}
//       />
//     </TableContainer>
//   );
// };

// export default Demo;
