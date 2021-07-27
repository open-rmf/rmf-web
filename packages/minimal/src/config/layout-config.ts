export interface LayoutConfig {
  noOfPanels: number;
  components: string[];
}

export const layoutConfig: LayoutConfig = (() => {
  const appUser = process.env.REACT_APP_USER;
  if (!appUser) {
    throw new Error('missing REACT_APP_USER');
  }
  if (appUser === 'admin') {
    return {
      noOfPanels: 2,
      components: ['Radio Group', 'Task Table'],
    };
  }
  return {
    noOfPanels: 1,
    components: ['Radio Group'],
  };
})();

export default layoutConfig;
