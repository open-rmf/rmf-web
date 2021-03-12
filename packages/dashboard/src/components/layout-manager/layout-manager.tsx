import { Mosaic, MosaicWindow, MosaicBranch } from 'react-mosaic-component';
import 'react-mosaic-component/react-mosaic-component.css';
import './layout-manager.css';
import Debug from 'debug';
import React from 'react';
import BuildingMapWindow from './building-map-window';

const debug = Debug('LayoutManager');

export default function LayoutManager(_props: {}): React.ReactElement {
  debug('render');

  function create_tile(id: string, path: MosaicBranch[]): JSX.Element {
    if (id === 'map_2d') return <BuildingMapWindow path={path} />;
    else {
      return (
        <MosaicWindow<string>
          path={path}
          className="layout-manager-theme"
          title={id}
          toolbarControls={[]}
        >
          <div>panel id: {id}</div>
        </MosaicWindow>
      );
    }
  }

  return (
    <div id="layout_manager">
      <Mosaic<string>
        renderTile={(id, path) => create_tile(id, path)}
        resize={{ minimumPaneSizePercentage: 5 }}
        initialValue={{
          direction: 'row',
          first: 'map_2d',
          second: {
            direction: 'column',
            first: 'placeholder_A',
            second: 'placeholder_B',
            splitPercentage: 40,
          },
          splitPercentage: 70,
        }}
      />
    </div>
  );
}
