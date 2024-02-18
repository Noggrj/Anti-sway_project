function printARX(C,d,in,out) 
% Imprime modelo ARX referente à função(e transferência discreta C;
% Parâmetros:
%  C - Função de transferência em z;
%  d - atraso extra de tempo para o modelo ARX.

   Cnum = tf(C).num{1};
   Cden = tf(C).den{1};

   flag = false;
   if(d==0)
      fprintf([out '(k) = ']);
   else
      fprintf([out '(k-' num2str(d) ') = ']);
   end
   for j=1:length(Cnum)
      coef = Cnum(j);
      if (coef ~= 0)
         if ( (coef >= 0) & flag )
            fprintf('+')
         end
         fprintf([num2str(coef) '*' in '(k-' num2str(j-1+d) ')']);
         flag = true;
      else
      end

   end
   for j=2:length(Cden)
      coef = -Cden(j);
      if (coef ~= 0)
         if (coef >= 0)
            fprintf('+')
         end
         fprintf([num2str(coef) '*' out '(k-' num2str(j-1+d) ')']);
      end
   end
   fprintf('\n')


