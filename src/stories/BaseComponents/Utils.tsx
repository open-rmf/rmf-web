import React from 'react';

export const viewBoxCoords: string = '0 0 25.794363144785166 14.53525484725833';

export interface styleTyping {
  [key: string]: React.CSSProperties;
}

export interface sampleStyleTyping {
  colorSample: {
    [key: string]: React.CSSProperties;
  }
}

export interface formProps {
  errorState: boolean[];
  errorMessage: string[];
  labels: string[];
}