export const getLogo = async () => {
  try {
    return await require(`../assets/defaultLogo.png`).default;
  } catch {
    return '';
  }
};
