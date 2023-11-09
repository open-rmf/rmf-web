import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import { LayersController } from './layers-controller';

describe('LayersController', () => {
  it('renders the component and handles interactions correctly', () => {
    const levels = [
      {
        name: 'L1',
        elevation: 0,
        images: [
          {
            name: 'office',
            x_offset: 0,
            y_offset: 0,
            yaw: 0,
            scale: 0.008465494960546494,
            encoding: 'png',
            data: '/assets/office.png',
          },
        ],
        places: [],
        doors: [
          {
            name: 'main_door',
            v1_x: 12.18443775177002,
            v1_y: -2.59969425201416,
            v2_x: 14.079463958740234,
            v2_y: -2.5592799186706543,
            door_type: 6,
            motion_range: 1.5707963705062866,
            motion_direction: 1,
          },
          {
            name: 'coe_door',
            v1_x: 8.256338119506836,
            v1_y: -5.49263334274292,
            v2_x: 7.8990349769592285,
            v2_y: -6.304050922393799,
            door_type: 5,
            motion_range: 1.5707963705062866,
            motion_direction: 1,
          },
          {
            name: 'hardware_door',
            v1_x: 19.447721481323242,
            v1_y: -10.76378345489502,
            v2_x: 19.452404022216797,
            v2_y: -9.874534606933594,
            door_type: 5,
            motion_range: 1.5707963705062866,
            motion_direction: 1,
          },
        ],
        nav_graphs: [
          {
            name: '0',
            vertices: [
              {
                x: 6.897922515869141,
                y: -2.025369644165039,
                name: '',
                params: [
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: false,
                  },
                ],
              },
              { x: 10.247854232788086, y: -3.0920557975769043, name: '', params: [] },
              { x: 16.855825424194336, y: -6.881363868713379, name: '', params: [] },
              {
                x: 18.739524841308594,
                y: -6.873981952667236,
                name: '',
                params: [
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: false,
                  },
                ],
              },
              {
                x: 16.84633445739746,
                y: -5.404067039489746,
                name: 'pantry',
                params: [
                  {
                    name: 'pickup_dispenser',
                    type: 1,
                    value_int: 0,
                    value_float: 0,
                    value_string: 'coke_dispenser',
                    value_bool: false,
                  },
                  {
                    name: 'is_holding_point',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: false,
                  },
                ],
              },
              { x: 13.130566596984863, y: -3.964806079864502, name: '', params: [] },
              { x: 8.91185474395752, y: -6.181352138519287, name: '', params: [] },
              { x: 10.086146354675293, y: -6.97943639755249, name: '', params: [] },
              {
                x: 7.914645195007324,
                y: -7.918869495391846,
                name: '',
                params: [
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: false,
                  },
                ],
              },
              { x: 18.8138427734375, y: -11.0789794921875, name: '', params: [] },
              { x: 18.794422149658203, y: -10.372406959533691, name: '', params: [] },
              { x: 7.9883036613464355, y: -10.780257225036621, name: '', params: [] },
              { x: 9.376501083374023, y: -11.142885208129883, name: '', params: [] },
              {
                x: 6.264034271240234,
                y: -3.515686273574829,
                name: 'supplies',
                params: [
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                ],
              },
              { x: 18.729019165039062, y: -3.895981550216675, name: '', params: [] },
              {
                x: 19.89569854736328,
                y: -3.4071500301361084,
                name: 'lounge',
                params: [
                  {
                    name: 'is_holding_point',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: false,
                  },
                ],
              },
              {
                x: 10.433053970336914,
                y: -5.5750956535339355,
                name: 'tinyRobot1_charger',
                params: [
                  {
                    name: 'is_charger',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                  {
                    name: 'is_holding_point',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                ],
              },
              { x: 11.565889358520508, y: -6.99680757522583, name: '', params: [] },
              { x: 15.298604965209961, y: -6.92883825302124, name: '', params: [] },
              { x: 11.55336856842041, y: -11.315971374511719, name: '', params: [] },
              { x: 11.573761940002441, y: -9.250288963317871, name: '', params: [] },
              { x: 15.15718936920166, y: -11.227091789245605, name: '', params: [] },
              { x: 15.166704177856445, y: -9.242974281311035, name: '', params: [] },
              { x: 6.517305374145508, y: -5.23292875289917, name: '', params: [] },
              {
                x: 5.346484661102295,
                y: -4.976813793182373,
                name: 'coe',
                params: [
                  {
                    name: 'is_holding_point',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: false,
                  },
                ],
              },
              { x: 17.067495346069336, y: -11.097883224487305, name: '', params: [] },
              { x: 20.893653869628906, y: -10.30837345123291, name: '', params: [] },
              {
                x: 20.9482479095459,
                y: -7.497346878051758,
                name: 'hardware_2',
                params: [
                  {
                    name: 'dropoff_ingestor',
                    type: 1,
                    value_int: 0,
                    value_float: 0,
                    value_string: 'coke_ingestor',
                    value_bool: false,
                  },
                  {
                    name: 'is_holding_point',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: false,
                  },
                ],
              },
              {
                x: 20.42369270324707,
                y: -5.312098026275635,
                name: 'tinyRobot2_charger',
                params: [
                  {
                    name: 'is_charger',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                  {
                    name: 'is_holding_point',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                  {
                    name: 'is_parking_spot',
                    type: 4,
                    value_int: 0,
                    value_float: 0,
                    value_string: '',
                    value_bool: true,
                  },
                ],
              },
            ],
            edges: [
              { v1_idx: 0, v2_idx: 1, params: [], edge_type: 0 },
              { v1_idx: 2, v2_idx: 3, params: [], edge_type: 0 },
              { v1_idx: 2, v2_idx: 4, params: [], edge_type: 0 },
              { v1_idx: 5, v2_idx: 1, params: [], edge_type: 0 },
              { v1_idx: 1, v2_idx: 6, params: [], edge_type: 0 },
              { v1_idx: 6, v2_idx: 7, params: [], edge_type: 0 },
              { v1_idx: 6, v2_idx: 8, params: [], edge_type: 0 },
              { v1_idx: 9, v2_idx: 10, params: [], edge_type: 0 },
              { v1_idx: 8, v2_idx: 11, params: [], edge_type: 0 },
              { v1_idx: 11, v2_idx: 12, params: [], edge_type: 0 },
              { v1_idx: 0, v2_idx: 13, params: [], edge_type: 0 },
              { v1_idx: 14, v2_idx: 5, params: [], edge_type: 0 },
              { v1_idx: 14, v2_idx: 15, params: [], edge_type: 0 },
              { v1_idx: 7, v2_idx: 16, params: [], edge_type: 0 },
              { v1_idx: 7, v2_idx: 17, params: [], edge_type: 0 },
              { v1_idx: 17, v2_idx: 18, params: [], edge_type: 0 },
              { v1_idx: 18, v2_idx: 2, params: [], edge_type: 0 },
              { v1_idx: 19, v2_idx: 20, params: [], edge_type: 0 },
              { v1_idx: 20, v2_idx: 17, params: [], edge_type: 0 },
              { v1_idx: 21, v2_idx: 22, params: [], edge_type: 0 },
              { v1_idx: 22, v2_idx: 18, params: [], edge_type: 0 },
              { v1_idx: 6, v2_idx: 23, params: [], edge_type: 0 },
              { v1_idx: 23, v2_idx: 24, params: [], edge_type: 0 },
              { v1_idx: 3, v2_idx: 10, params: [], edge_type: 0 },
              { v1_idx: 9, v2_idx: 25, params: [], edge_type: 0 },
              { v1_idx: 10, v2_idx: 26, params: [], edge_type: 0 },
              { v1_idx: 26, v2_idx: 27, params: [], edge_type: 0 },
              { v1_idx: 12, v2_idx: 19, params: [], edge_type: 0 },
              { v1_idx: 19, v2_idx: 21, params: [], edge_type: 0 },
              { v1_idx: 21, v2_idx: 25, params: [], edge_type: 0 },
              { v1_idx: 14, v2_idx: 28, params: [], edge_type: 0 },
              { v1_idx: 14, v2_idx: 3, params: [], edge_type: 0 },
            ],
            params: [],
          },
        ],
        wall_graph: { name: '', vertices: [], edges: [], params: [] },
      },
    ];

    const onChangeMock = jest.fn();
    const handleZoomInMock = jest.fn();
    const handleZoomOutMock = jest.fn();

    const disabledLayers = {
      Layer1: false,
      Layer2: true,
      Layer3: false,
    };

    const { getByText, getByLabelText, getByTestId } = render(
      <LayersController
        disabledLayers={disabledLayers}
        onChange={onChangeMock}
        levels={levels}
        currentLevel={levels[0]}
        handleZoomIn={handleZoomInMock}
        handleZoomOut={handleZoomOutMock}
      />,
    );

    expect(getByLabelText('Levels')).toBeTruthy();

    fireEvent.click(getByTestId('zoom-in'));
    expect(handleZoomInMock).toHaveBeenCalled();

    fireEvent.click(getByTestId('zoom-out'));

    expect(handleZoomOutMock).toHaveBeenCalled();

    fireEvent.mouseEnter(getByTestId('layers'));

    expect(getByText('Layer1')).toBeTruthy();
    expect(getByText('Layer2')).toBeTruthy();
    expect(getByText('Layer3')).toBeTruthy();
  });
});
