const { getDefaultConfig } = require('expo/metro-config');

/** @type {import('expo/metro-config').MetroConfig} */
const config = getDefaultConfig(__dirname);

// Enable package.json "exports" field resolution for better tree-shaking
config.resolver.unstable_enablePackageExports = true;

module.exports = config;
