import { render as render_, RenderOptions } from '@testing-library/react';
import React from 'react';
import { Map as LMap } from 'react-leaflet';

function Wrapper({ children }: React.PropsWithChildren<{}>) {
  return (
    <LMap
      bounds={[
        [0, 0],
        [1, 1],
      ]}
    >
      {children}
    </LMap>
  );
}

export function render(ui: React.ReactElement, options?: Omit<RenderOptions, 'wrapper'>) {
  return render_(ui, { wrapper: Wrapper, ...options });
}
