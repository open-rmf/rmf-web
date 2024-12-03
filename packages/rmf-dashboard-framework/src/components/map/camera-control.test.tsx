import { Canvas } from '@react-three/fiber';
import { render } from '@testing-library/react';
import React from 'react';
import { describe, it } from 'vitest';

import { TestProviders } from '../../utils/test-utils.test';
import { CameraControl } from './camera-control';

describe('CameraControl', () => {
  const Base = (_: React.PropsWithChildren<{}>) => {
    return <TestProviders />;
  };

  it('should render without crashing', () => {
    render(
      <Base>
        <Canvas>
          <CameraControl zoom={1} />
        </Canvas>
      </Base>,
    );
  });
});
