function varargout = conic_option_info(varargin)
    %CONIC_OPTION_INFO [INTERNAL] 
    %
    %  char = CONIC_OPTION_INFO(char name, char op)
    %
    %Get documentation for a particular option.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1em
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L564
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L564-L566
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(823, varargin{:});
end
