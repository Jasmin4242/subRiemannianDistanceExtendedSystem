function YlYiYjyk_p =  get_YlYiYjyk_p(vektorfeld_rechts,vektorfeld_links,vektorfeld_ganz_links,funktion,y,p,n)

    %zuerst zB Y2y4 bestimmen
    sym ergebnis_schritt1;
    ergebnis_schritt1=0;
    for ii=1:n
        ergebnis_schritt1=ergebnis_schritt1+vektorfeld_rechts(ii)*diff(funktion,y(ii));
    end
    
    %dann Y1Y2y4 bestimmen
    sym ergebnis_schritt2;
    ergebnis_schritt2=0;
    for ii=1:n
        ergebnis_schritt2=ergebnis_schritt2+vektorfeld_links(ii)*diff(ergebnis_schritt1,y(ii));
    end
    
    %Y1Y1Y2y4
    sym ergebnis_schritt3;
    ergebnis_schritt3=0;
    for ii=1:n
        ergebnis_schritt3=ergebnis_schritt3+vektorfeld_ganz_links(ii)*diff(ergebnis_schritt2,y(ii));
    end  
        
    YlYiYjyk_p=subs(ergebnis_schritt3,[y(1) y(2) y(3) y(4)],[p(1) p(2) p(3) p(4)]);

end